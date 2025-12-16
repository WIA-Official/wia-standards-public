# WIA Voice-Sign Monitoring Specification

Version: 1.0.0
Status: Draft
Last Updated: 2025-01-15

## 1. Overview

This specification defines the monitoring, observability, and alerting standards for WIA Voice-Sign translation systems.

## 2. Health Check Endpoints

### 2.1 Endpoint Definitions

```yaml
health_endpoints:
  /health:
    method: GET
    description: Combined health status
    response:
      status: healthy | degraded | unhealthy
      components: {}
      timestamp: ISO8601

  /health/live:
    method: GET
    description: Liveness probe (is the process running?)
    response:
      status: ok | fail

  /health/ready:
    method: GET
    description: Readiness probe (can accept traffic?)
    response:
      status: ready | not_ready
      checks: {}

  /health/startup:
    method: GET
    description: Startup probe (has initialization completed?)
    response:
      status: started | starting | failed
```

### 2.2 Health Check Response Format

```json
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-01-15T10:30:00Z",
  "uptime_seconds": 86400,
  "components": {
    "database": {
      "status": "healthy",
      "latency_ms": 5,
      "last_check": "2025-01-15T10:29:55Z"
    },
    "redis": {
      "status": "healthy",
      "latency_ms": 1,
      "last_check": "2025-01-15T10:29:55Z"
    },
    "asr_model": {
      "status": "healthy",
      "model_version": "whisper-v3",
      "gpu_available": true
    },
    "translation_model": {
      "status": "healthy",
      "model_version": "1.0.0",
      "languages_loaded": ["ASL", "BSL", "KSL"]
    }
  },
  "checks": {
    "disk_space": {
      "status": "pass",
      "available_gb": 45.2
    },
    "memory": {
      "status": "pass",
      "used_percent": 62.5
    },
    "cpu": {
      "status": "pass",
      "load_average": 1.5
    }
  }
}
```

### 2.3 Status Codes

| Endpoint | Success | Degraded | Failure |
|----------|---------|----------|---------|
| /health | 200 | 200 | 503 |
| /health/live | 200 | - | 503 |
| /health/ready | 200 | - | 503 |
| /health/startup | 200 | - | 503 |

## 3. Metrics Collection

### 3.1 Prometheus Metrics

#### HTTP Metrics
```prometheus
# Request count by endpoint and status
voice_sign_http_requests_total{method="POST", endpoint="/translate", status="200"} 1234

# Request duration histogram
voice_sign_http_request_duration_seconds_bucket{endpoint="/translate", le="0.1"} 900
voice_sign_http_request_duration_seconds_bucket{endpoint="/translate", le="0.5"} 1200
voice_sign_http_request_duration_seconds_bucket{endpoint="/translate", le="1.0"} 1230
voice_sign_http_request_duration_seconds_sum{endpoint="/translate"} 456.78
voice_sign_http_request_duration_seconds_count{endpoint="/translate"} 1234

# Request size
voice_sign_http_request_size_bytes{endpoint="/translate", quantile="0.5"} 1024
voice_sign_http_request_size_bytes{endpoint="/translate", quantile="0.99"} 5242880

# Active connections
voice_sign_http_active_connections 45
```

#### Translation Metrics
```prometheus
# Translation request count
voice_sign_translations_total{source_lang="en", target_lang="ASL", status="success"} 5000

# Translation duration by stage
voice_sign_translation_stage_duration_seconds{stage="asr", quantile="0.5"} 0.15
voice_sign_translation_stage_duration_seconds{stage="translation", quantile="0.5"} 0.08
voice_sign_translation_stage_duration_seconds{stage="pose_generation", quantile="0.5"} 0.12
voice_sign_translation_stage_duration_seconds{stage="rendering", quantile="0.5"} 0.25

# Translation quality scores
voice_sign_translation_quality_score{target_lang="ASL", quantile="0.5"} 0.92
voice_sign_translation_quality_score{target_lang="ASL", quantile="0.99"} 0.75

# Queue depth
voice_sign_translation_queue_depth{priority="normal"} 15
voice_sign_translation_queue_depth{priority="emergency"} 0

# Cache metrics
voice_sign_cache_hits_total{cache="translation"} 10000
voice_sign_cache_misses_total{cache="translation"} 500
voice_sign_cache_size_bytes{cache="translation"} 52428800
```

#### ASR Metrics
```prometheus
# ASR processing
voice_sign_asr_processing_total{language="en", status="success"} 4500
voice_sign_asr_processing_duration_seconds{language="en", quantile="0.5"} 0.12

# Audio metrics
voice_sign_audio_duration_seconds{quantile="0.5"} 5.0
voice_sign_audio_duration_seconds{quantile="0.99"} 45.0

# Word error rate
voice_sign_asr_word_error_rate{language="en", quantile="0.5"} 0.05

# Voice activity detection
voice_sign_vad_speech_ratio{quantile="0.5"} 0.75
```

#### Safety Metrics
```prometheus
# Content filter actions
voice_sign_content_filter_total{category="profanity", action="censor"} 50
voice_sign_content_filter_total{category="violence", action="warn"} 10
voice_sign_content_filter_total{category="discrimination", action="block"} 2

# Emergency detection
voice_sign_emergency_detected_total{urgency="critical"} 5
voice_sign_emergency_detected_total{urgency="high"} 12

# PII detection
voice_sign_pii_detected_total{type="email"} 100
voice_sign_pii_redacted_total 95
```

#### System Metrics
```prometheus
# Resource usage
voice_sign_cpu_usage_percent 45.5
voice_sign_memory_usage_bytes 1073741824
voice_sign_disk_usage_bytes 10737418240

# GPU metrics (if available)
voice_sign_gpu_utilization_percent{gpu="0"} 78.5
voice_sign_gpu_memory_used_bytes{gpu="0"} 8589934592
voice_sign_gpu_temperature_celsius{gpu="0"} 65

# Model metrics
voice_sign_model_load_time_seconds{model="asr"} 5.2
voice_sign_model_inference_time_seconds{model="asr", quantile="0.5"} 0.1
```

### 3.2 Metric Labels

| Label | Description | Example Values |
|-------|-------------|----------------|
| endpoint | API endpoint | /translate, /health |
| method | HTTP method | GET, POST |
| status | HTTP status code | 200, 400, 500 |
| source_lang | Source language | en, ko, ja |
| target_lang | Target sign language | ASL, BSL, KSL |
| stage | Processing stage | asr, translation, rendering |
| priority | Request priority | normal, high, emergency |
| cache | Cache name | translation, gloss |
| model | Model identifier | asr, translation |

### 3.3 Metrics Endpoint

```yaml
metrics:
  endpoint: /metrics
  port: 9090
  format: prometheus

  authentication:
    enabled: false  # Internal network only

  filters:
    - prefix: voice_sign_
    - prefix: go_
    - prefix: process_
```

## 4. Logging

### 4.1 Log Format

```json
{
  "timestamp": "2025-01-15T10:30:00.123Z",
  "level": "INFO",
  "service": "voice-sign-api",
  "version": "1.0.0",
  "instance_id": "api-pod-abc123",
  "trace_id": "4bf92f3577b34da6a3ce929d0e0e4736",
  "span_id": "00f067aa0ba902b7",
  "request_id": "req-12345",
  "message": "Translation completed",
  "fields": {
    "source_language": "en",
    "target_language": "ASL",
    "duration_ms": 250,
    "quality_score": 0.95
  }
}
```

### 4.2 Log Levels

| Level | Description | Examples |
|-------|-------------|----------|
| TRACE | Detailed debugging | Function entry/exit, variable values |
| DEBUG | Debug information | Request details, intermediate results |
| INFO | Normal operations | Translation completed, model loaded |
| WARN | Warning conditions | Low confidence, rate limit approaching |
| ERROR | Error conditions | Translation failed, model error |
| FATAL | System failure | Startup failed, unrecoverable error |

### 4.3 Log Categories

```rust
// Translation logs
log::info!(target: "translation", "Translation completed";
    "request_id" => request_id,
    "duration_ms" => duration,
    "quality_score" => score
);

// Safety logs
log::warn!(target: "safety", "Content filtered";
    "request_id" => request_id,
    "category" => "profanity",
    "action" => "censor"
);

// Performance logs
log::debug!(target: "performance", "Stage completed";
    "request_id" => request_id,
    "stage" => "asr",
    "duration_ms" => asr_duration
);

// Security logs
log::warn!(target: "security", "Rate limit exceeded";
    "client_id" => client_id,
    "requests" => request_count,
    "limit" => rate_limit
);
```

### 4.4 Log Aggregation

```yaml
log_aggregation:
  collector: fluentd  # or fluent-bit, vector

  outputs:
    - type: elasticsearch
      index: voice-sign-logs-{date}
      retention: 30d

    - type: s3
      bucket: wia-logs
      prefix: voice-sign/
      retention: 365d

  parsing:
    format: json
    timestamp_field: timestamp
    timestamp_format: ISO8601

  enrichment:
    - kubernetes_metadata
    - geo_ip
```

## 5. Distributed Tracing

### 5.1 OpenTelemetry Configuration

```yaml
opentelemetry:
  service_name: voice-sign-api
  service_version: "1.0.0"

  exporter:
    type: otlp
    endpoint: http://otel-collector:4317
    protocol: grpc

  sampling:
    type: probability
    rate: 0.1  # 10% of traces

  propagation:
    - traceparent
    - tracestate
    - baggage

  resource_attributes:
    service.namespace: wia
    deployment.environment: production
```

### 5.2 Trace Spans

```rust
// Translation request span
#[instrument(
    name = "translate_request",
    skip(self, request),
    fields(
        request_id = %request.request_id,
        target_language = %request.target_language
    )
)]
async fn translate(&self, request: TranslationRequest) -> Result<TranslationResponse> {
    // ASR span
    let transcript = {
        let _span = tracing::info_span!("asr_transcribe").entered();
        self.asr.transcribe(&request.audio).await?
    };

    // Translation span
    let gloss = {
        let _span = tracing::info_span!("translate_to_sign").entered();
        self.translator.translate(&transcript).await?
    };

    // Pose generation span
    let pose = {
        let _span = tracing::info_span!("generate_pose").entered();
        self.pose_generator.generate(&gloss).await?
    };

    // Rendering span (if requested)
    let render = if request.output.include_render {
        let _span = tracing::info_span!("render_avatar").entered();
        Some(self.renderer.render(&pose).await?)
    } else {
        None
    };

    Ok(TranslationResponse { ... })
}
```

### 5.3 Trace Context

```yaml
trace_context:
  headers:
    - traceparent  # W3C Trace Context
    - X-Request-ID  # Request correlation

  baggage:
    - client_id
    - user_id
    - session_id

  attributes:
    - http.method
    - http.url
    - http.status_code
    - http.user_agent
    - net.peer.ip
```

## 6. Alerting

### 6.1 Alert Rules

```yaml
groups:
  - name: voice-sign-availability
    rules:
      - alert: VoiceSignApiDown
        expr: up{job="voice-sign-api"} == 0
        for: 1m
        labels:
          severity: critical
        annotations:
          summary: "Voice-Sign API is down"
          description: "Voice-Sign API instance {{ $labels.instance }} is down for more than 1 minute"

      - alert: VoiceSignHighErrorRate
        expr: |
          sum(rate(voice_sign_http_requests_total{status=~"5.."}[5m]))
          / sum(rate(voice_sign_http_requests_total[5m])) > 0.01
        for: 5m
        labels:
          severity: critical
        annotations:
          summary: "High error rate detected"
          description: "Error rate is {{ $value | humanizePercentage }}"

  - name: voice-sign-latency
    rules:
      - alert: VoiceSignHighLatency
        expr: |
          histogram_quantile(0.99,
            sum(rate(voice_sign_http_request_duration_seconds_bucket[5m])) by (le)
          ) > 1
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "High latency detected"
          description: "P99 latency is {{ $value }}s"

      - alert: VoiceSignTranslationSlow
        expr: |
          histogram_quantile(0.95,
            sum(rate(voice_sign_translation_stage_duration_seconds_bucket[5m])) by (le, stage)
          ) > 0.5
        for: 10m
        labels:
          severity: warning
        annotations:
          summary: "Translation stage {{ $labels.stage }} is slow"

  - name: voice-sign-quality
    rules:
      - alert: VoiceSignLowQualityScore
        expr: |
          histogram_quantile(0.5,
            sum(rate(voice_sign_translation_quality_score_bucket[1h])) by (le)
          ) < 0.8
        for: 30m
        labels:
          severity: warning
        annotations:
          summary: "Translation quality is degraded"
          description: "Median quality score is {{ $value }}"

  - name: voice-sign-resources
    rules:
      - alert: VoiceSignHighCpuUsage
        expr: voice_sign_cpu_usage_percent > 85
        for: 10m
        labels:
          severity: warning
        annotations:
          summary: "High CPU usage"
          description: "CPU usage is {{ $value }}%"

      - alert: VoiceSignHighMemoryUsage
        expr: voice_sign_memory_usage_bytes / voice_sign_memory_limit_bytes > 0.9
        for: 10m
        labels:
          severity: warning
        annotations:
          summary: "High memory usage"

      - alert: VoiceSignGpuMemoryHigh
        expr: voice_sign_gpu_memory_used_bytes / voice_sign_gpu_memory_total_bytes > 0.95
        for: 5m
        labels:
          severity: critical
        annotations:
          summary: "GPU memory nearly exhausted"

  - name: voice-sign-emergency
    rules:
      - alert: VoiceSignEmergencyBacklog
        expr: voice_sign_translation_queue_depth{priority="emergency"} > 0
        for: 30s
        labels:
          severity: critical
        annotations:
          summary: "Emergency translations queued"
          description: "{{ $value }} emergency translations waiting"
```

### 6.2 Alert Severity Levels

| Severity | Response Time | Notification | Examples |
|----------|---------------|--------------|----------|
| Critical | Immediate | PagerDuty + Slack | Service down, data loss risk |
| Warning | 15 minutes | Slack | High latency, resource pressure |
| Info | Next business day | Email | Capacity planning, trends |

### 6.3 Alert Routing

```yaml
route:
  group_by: ['alertname', 'severity']
  group_wait: 30s
  group_interval: 5m
  repeat_interval: 4h
  receiver: 'default'

  routes:
    - match:
        severity: critical
      receiver: 'pagerduty-critical'
      continue: true

    - match:
        severity: critical
      receiver: 'slack-critical'

    - match:
        severity: warning
      receiver: 'slack-warning'

receivers:
  - name: 'pagerduty-critical'
    pagerduty_configs:
      - service_key: <key>
        severity: critical

  - name: 'slack-critical'
    slack_configs:
      - channel: '#voice-sign-alerts'
        send_resolved: true

  - name: 'slack-warning'
    slack_configs:
      - channel: '#voice-sign-monitoring'
        send_resolved: true
```

## 7. Performance Benchmarks

### 7.1 Latency Targets

| Operation | P50 | P95 | P99 | Max |
|-----------|-----|-----|-----|-----|
| Health check | 5ms | 10ms | 20ms | 100ms |
| ASR (10s audio) | 150ms | 300ms | 500ms | 1s |
| Translation | 50ms | 100ms | 200ms | 500ms |
| Pose generation | 100ms | 200ms | 300ms | 500ms |
| Full pipeline | 300ms | 600ms | 1s | 2s |
| Avatar rendering | 200ms | 400ms | 600ms | 1s |

### 7.2 Throughput Targets

| Metric | Target | Maximum |
|--------|--------|---------|
| Requests/second | 1000 | 5000 |
| Concurrent connections | 10000 | 50000 |
| Audio processing (minutes/sec) | 60 | 300 |
| Translations/second | 500 | 2000 |

### 7.3 Resource Efficiency

| Resource | Target Utilization | Alert Threshold |
|----------|-------------------|-----------------|
| CPU | 40-60% | 85% |
| Memory | 50-70% | 90% |
| GPU | 60-80% | 95% |
| Network | 30-50% | 80% |

## 8. SLA Definitions

### 8.1 Service Level Objectives (SLOs)

```yaml
slos:
  - name: availability
    description: "Service availability"
    target: 99.95%
    window: 30d
    indicator:
      type: availability
      good_events: "up == 1"
      total_events: "1"

  - name: latency
    description: "Translation latency"
    target: 99%
    window: 30d
    indicator:
      type: latency
      threshold: 1s
      percentile: 99

  - name: error_rate
    description: "Error rate"
    target: 99.9%
    window: 30d
    indicator:
      type: error_rate
      good_events: "status < 500"
      total_events: "all requests"

  - name: quality
    description: "Translation quality"
    target: 95%
    window: 7d
    indicator:
      type: quality
      threshold: 0.85
      metric: quality_score
```

### 8.2 Error Budget

```yaml
error_budget:
  availability:
    budget_minutes_per_month: 21.6  # 99.95% = 21.6 min downtime
    burn_rate_alert: 14.4  # 1 day budget in 1 hour
    burn_rate_warning: 6    # 1 day budget in 4 hours

  latency:
    budget_violations_per_month: 1%
    burn_rate_alert: 10x
    burn_rate_warning: 5x
```

## 9. Dashboard Panels

### 9.1 Overview Dashboard

```yaml
dashboard:
  title: "Voice-Sign Overview"

  rows:
    - title: "Service Health"
      panels:
        - type: stat
          title: "Uptime"
          query: "avg(up{job='voice-sign-api'})"

        - type: stat
          title: "Request Rate"
          query: "sum(rate(voice_sign_http_requests_total[5m]))"

        - type: stat
          title: "Error Rate"
          query: "sum(rate(voice_sign_http_requests_total{status=~'5..'}[5m])) / sum(rate(voice_sign_http_requests_total[5m]))"

    - title: "Latency"
      panels:
        - type: graph
          title: "Request Latency"
          queries:
            - "histogram_quantile(0.50, sum(rate(voice_sign_http_request_duration_seconds_bucket[5m])) by (le))"
            - "histogram_quantile(0.95, sum(rate(voice_sign_http_request_duration_seconds_bucket[5m])) by (le))"
            - "histogram_quantile(0.99, sum(rate(voice_sign_http_request_duration_seconds_bucket[5m])) by (le))"

    - title: "Translation Metrics"
      panels:
        - type: graph
          title: "Translations by Language"
          query: "sum(rate(voice_sign_translations_total[5m])) by (target_lang)"

        - type: graph
          title: "Quality Scores"
          query: "histogram_quantile(0.5, sum(rate(voice_sign_translation_quality_score_bucket[5m])) by (le, target_lang))"

    - title: "Resources"
      panels:
        - type: graph
          title: "CPU Usage"
          query: "voice_sign_cpu_usage_percent"

        - type: graph
          title: "Memory Usage"
          query: "voice_sign_memory_usage_bytes"

        - type: graph
          title: "GPU Utilization"
          query: "voice_sign_gpu_utilization_percent"
```

### 9.2 Alert Configuration

```yaml
alerting:
  datasources:
    - name: Prometheus
      type: prometheus
      url: http://prometheus:9090

  notification_channels:
    - name: Slack
      type: slack
      settings:
        url: https://hooks.slack.com/...

    - name: PagerDuty
      type: pagerduty
      settings:
        integrationKey: ...
```
