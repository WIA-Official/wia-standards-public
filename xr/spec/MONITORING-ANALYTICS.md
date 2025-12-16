# WIA XR Accessibility: Monitoring & Analytics Specification

## 1. Overview

본 문서는 XR 접근성 시스템의 모니터링 및 분석 표준을 정의합니다.

**Version:** 1.0.0
**Status:** Draft
**Phase:** 4 - Integration & Deployment Protocol

---

## 2. Monitoring Architecture

### 2.1 Monitoring Stack

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     WIA XR Monitoring Architecture                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────┐        │
│  │                     Data Collection Layer                    │        │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │        │
│  │  │ XR App  │  │ XR SDK  │  │ WIA Hub │  │ Devices │        │        │
│  │  │ Metrics │  │ Metrics │  │ Metrics │  │ Metrics │        │        │
│  │  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘        │        │
│  └───────┴────────────┴────────────┴────────────┴─────────────┘        │
│                              │                                          │
│                       ┌──────▼──────┐                                   │
│                       │  Collector  │                                   │
│                       │  (OTel)     │                                   │
│                       └──────┬──────┘                                   │
│                              │                                          │
│          ┌───────────────────┼───────────────────┐                      │
│          │                   │                   │                      │
│   ┌──────▼──────┐     ┌──────▼──────┐     ┌──────▼──────┐              │
│   │   Metrics   │     │    Logs     │     │   Traces    │              │
│   │ (Prometheus)│     │ (Loki/ES)   │     │  (Jaeger)   │              │
│   └──────┬──────┘     └──────┬──────┘     └──────┬──────┘              │
│          │                   │                   │                      │
│          └───────────────────┼───────────────────┘                      │
│                              │                                          │
│                       ┌──────▼──────┐                                   │
│                       │  Analytics  │                                   │
│                       │   Engine    │                                   │
│                       └──────┬──────┘                                   │
│                              │                                          │
│          ┌───────────────────┼───────────────────┐                      │
│          │                   │                   │                      │
│   ┌──────▼──────┐     ┌──────▼──────┐     ┌──────▼──────┐              │
│   │ Dashboards  │     │   Alerts    │     │  Reports    │              │
│   │  (Grafana)  │     │ (PagerDuty) │     │  (Custom)   │              │
│   └─────────────┘     └─────────────┘     └─────────────┘              │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Metrics Categories

```typescript
interface XRMetricsCategories {
  // 시스템 메트릭
  system: {
    frameRate: Metric;
    latency: Metric;
    memoryUsage: Metric;
    cpuUsage: Metric;
    gpuUsage: Metric;
    batteryLevel: Metric;
    thermalState: Metric;
  };

  // 접근성 메트릭
  accessibility: {
    featureUsage: Metric;
    adaptationCount: Metric;
    captionAccuracy: Metric;
    audioDescriptionUsage: Metric;
    screenReaderActivity: Metric;
    inputMethodUsage: Metric;
  };

  // 사용자 경험 메트릭
  userExperience: {
    sessionDuration: Metric;
    taskCompletionRate: Metric;
    errorRate: Metric;
    userSatisfaction: Metric;
    accessibilityScore: Metric;
  };

  // WIA 기기 메트릭
  wiaDevices: {
    connectionStatus: Metric;
    syncLatency: Metric;
    commandSuccessRate: Metric;
    healthStatus: Metric;
  };

  // 비즈니스 메트릭
  business: {
    activeUsers: Metric;
    profileCreations: Metric;
    certificationLevel: Metric;
    featureAdoption: Metric;
  };
}
```

---

## 3. Core Metrics

### 3.1 Performance Metrics

```yaml
performance_metrics:
  frame_rate:
    name: xr_frame_rate_fps
    type: gauge
    description: "Current frame rate"
    labels: [platform, device_model]
    thresholds:
      critical: 60
      warning: 72
      target: 90

  motion_to_photon_latency:
    name: xr_mtp_latency_ms
    type: histogram
    description: "Motion-to-photon latency"
    buckets: [10, 15, 20, 25, 30, 50]
    labels: [platform]
    slo: p99 < 20ms

  adaptation_overhead:
    name: xr_adaptation_overhead_percent
    type: gauge
    description: "Performance overhead from accessibility features"
    labels: [adaptation_type]
    threshold: < 10%

  profile_load_time:
    name: xr_profile_load_duration_ms
    type: histogram
    description: "Time to load and apply profile"
    buckets: [50, 100, 200, 500, 1000]
    slo: p95 < 500ms
```

### 3.2 Accessibility Metrics

```yaml
accessibility_metrics:
  feature_usage:
    name: xr_accessibility_feature_usage
    type: counter
    description: "Usage count by accessibility feature"
    labels:
      - feature_type: [captions, audio_description, screen_reader, magnification, voice_control, eye_tracking]
      - disability_category: [visual, auditory, motor, cognitive]
      - enabled: [true, false]

  caption_metrics:
    - name: xr_caption_display_count
      type: counter
      description: "Number of captions displayed"

    - name: xr_caption_sync_accuracy_percent
      type: gauge
      description: "Caption synchronization accuracy"
      target: ">= 95%"

    - name: xr_caption_word_accuracy_percent
      type: gauge
      description: "Caption word accuracy"
      target: ">= 98%"

  audio_description_metrics:
    - name: xr_audio_description_played_count
      type: counter
      description: "Audio descriptions played"

    - name: xr_audio_description_skipped_count
      type: counter
      description: "Audio descriptions skipped by user"

  input_method_metrics:
    name: xr_input_method_usage
    type: counter
    labels: [method, success]
    methods:
      - voice_control
      - eye_tracking
      - hand_tracking
      - switch_access
      - head_tracking
```

### 3.3 Safety Metrics

```yaml
safety_metrics:
  session_health:
    - name: xr_session_duration_minutes
      type: histogram
      buckets: [15, 30, 60, 90, 120, 180]

    - name: xr_break_reminder_triggered_count
      type: counter

    - name: xr_break_taken_count
      type: counter

    - name: xr_session_fatigue_score
      type: gauge
      range: [0, 100]

  emergency_events:
    - name: xr_emergency_exit_triggered
      type: counter
      labels: [trigger_method]

    - name: xr_safe_space_activated
      type: counter

  comfort_metrics:
    - name: xr_motion_sickness_report_count
      type: counter

    - name: xr_comfort_setting_change_count
      type: counter
      labels: [setting_type]
```

### 3.4 WIA Device Metrics

```yaml
wia_device_metrics:
  connection:
    - name: wia_device_connection_status
      type: gauge
      labels: [device_type, device_id]
      values: [0=disconnected, 1=connecting, 2=connected, 3=error]

    - name: wia_device_connection_duration_seconds
      type: histogram
      buckets: [1, 5, 10, 30, 60]

    - name: wia_device_reconnection_count
      type: counter
      labels: [device_type]

  sync:
    - name: wia_sync_latency_ms
      type: histogram
      buckets: [10, 20, 50, 100, 200]
      labels: [device_type]

    - name: wia_sync_success_rate
      type: gauge
      labels: [device_type]

  exoskeleton_specific:
    - name: wia_exoskeleton_haptic_commands
      type: counter
      labels: [target, intensity_level]

    - name: wia_exoskeleton_safety_stop_count
      type: counter

  bionic_eye_specific:
    - name: wia_bionic_eye_stimulation_time_seconds
      type: counter

    - name: wia_bionic_eye_rest_time_seconds
      type: counter
```

---

## 4. Logging Standards

### 4.1 Log Levels

```yaml
log_levels:
  error:
    description: "System errors requiring immediate attention"
    examples:
      - "WIA device connection failed"
      - "Profile load critical error"
      - "Safety system triggered"
    retention: 90_days
    alert: immediate

  warn:
    description: "Warning conditions that may need attention"
    examples:
      - "Low frame rate detected"
      - "Adaptation fallback activated"
      - "High latency to WIA device"
    retention: 30_days
    alert: aggregated

  info:
    description: "Normal operational events"
    examples:
      - "Profile loaded successfully"
      - "Adaptation applied"
      - "Session started"
    retention: 14_days
    alert: none

  debug:
    description: "Detailed debugging information"
    examples:
      - "Adaptation calculation details"
      - "Event handler execution"
    retention: 7_days
    alert: none

  trace:
    description: "Very detailed tracing (development only)"
    retention: 1_day
    production: disabled
```

### 4.2 Log Schema

```typescript
interface XRLogEntry {
  // 필수 필드
  timestamp: string;          // ISO 8601
  level: LogLevel;
  message: string;
  service: string;

  // 컨텍스트
  context: {
    requestId?: string;
    sessionId?: string;
    userId?: string;          // 익명화됨
    platform: string;
    deviceModel?: string;
  };

  // 접근성 관련
  accessibility?: {
    activeAdaptations?: string[];
    disabilityCategories?: string[];  // 익명화됨
    profileVersion?: string;
  };

  // WIA 기기
  wiaDevice?: {
    deviceType?: string;
    deviceId?: string;
    connectionState?: string;
  };

  // 에러 정보
  error?: {
    code: string;
    message: string;
    stack?: string;
    recoverable: boolean;
  };

  // 메타데이터
  metadata?: Record<string, any>;
}
```

### 4.3 Structured Logging Examples

```json
// Profile loaded successfully
{
  "timestamp": "2025-01-15T10:30:00.000Z",
  "level": "info",
  "message": "Profile loaded successfully",
  "service": "xr-accessibility-engine",
  "context": {
    "requestId": "req_abc123",
    "sessionId": "sess_xyz789",
    "userId": "usr_hash_456",
    "platform": "quest3"
  },
  "accessibility": {
    "activeAdaptations": ["captions", "audio_description", "vignette"],
    "disabilityCategories": ["visual", "auditory"],
    "profileVersion": "1.2.0"
  },
  "metadata": {
    "loadTimeMs": 145,
    "adaptationsApplied": 3
  }
}
```

```json
// WIA device error
{
  "timestamp": "2025-01-15T10:35:00.000Z",
  "level": "error",
  "message": "WIA exoskeleton connection lost",
  "service": "wia-gateway",
  "context": {
    "sessionId": "sess_xyz789",
    "platform": "quest3"
  },
  "wiaDevice": {
    "deviceType": "exoskeleton",
    "deviceId": "exo_001",
    "connectionState": "disconnected"
  },
  "error": {
    "code": "WIA_CONNECTION_LOST",
    "message": "Bluetooth connection timeout after 30s",
    "recoverable": true
  },
  "metadata": {
    "reconnectAttempts": 3,
    "lastSuccessfulSync": "2025-01-15T10:34:30.000Z"
  }
}
```

---

## 5. Analytics Events

### 5.1 Event Schema

```typescript
interface AnalyticsEvent {
  // 이벤트 식별
  eventId: string;
  eventName: string;
  eventVersion: string;
  timestamp: string;

  // 사용자 (익명화)
  user: {
    anonymousId: string;
    sessionId: string;
    isNew: boolean;
  };

  // 컨텍스트
  context: {
    platform: string;
    appVersion: string;
    sdkVersion: string;
    locale: string;
    timezone: string;
  };

  // 접근성 컨텍스트
  accessibilityContext: {
    profileId: string;           // 해시됨
    disabilityCategories: string[];
    activeAdaptations: string[];
    certificationLevel?: string;
  };

  // 이벤트별 속성
  properties: Record<string, any>;
}
```

### 5.2 Core Events

```yaml
analytics_events:
  # 세션 이벤트
  session_events:
    - name: session_started
      properties:
        - platform
        - device_model
        - accessibility_features_enabled

    - name: session_ended
      properties:
        - duration_minutes
        - breaks_taken
        - fatigue_score

  # 프로필 이벤트
  profile_events:
    - name: profile_created
      properties:
        - disability_categories
        - adaptations_selected
        - wia_devices_configured

    - name: profile_loaded
      properties:
        - source: [local, cloud, qr, wia_hub]
        - load_time_ms

    - name: profile_updated
      properties:
        - changed_settings
        - trigger: [user, automatic, wia_sync]

  # 적응 이벤트
  adaptation_events:
    - name: adaptation_applied
      properties:
        - adaptation_type
        - trigger
        - success

    - name: adaptation_removed
      properties:
        - adaptation_type
        - reason

    - name: adaptation_fallback
      properties:
        - original_adaptation
        - fallback_used
        - reason

  # 접근성 기능 이벤트
  accessibility_events:
    - name: caption_displayed
      properties:
        - word_count
        - duration_ms
        - sync_accuracy

    - name: audio_description_played
      properties:
        - description_type
        - duration_ms
        - skipped

    - name: voice_command_used
      properties:
        - command_type
        - success
        - recognition_confidence

    - name: eye_tracking_interaction
      properties:
        - interaction_type
        - dwell_time_ms

  # WIA 기기 이벤트
  wia_events:
    - name: wia_device_connected
      properties:
        - device_type
        - connection_method
        - connection_time_ms

    - name: wia_device_disconnected
      properties:
        - device_type
        - reason
        - session_duration

    - name: wia_sync_completed
      properties:
        - device_types
        - settings_synced

  # 안전 이벤트
  safety_events:
    - name: emergency_exit_triggered
      properties:
        - trigger_method
        - session_duration

    - name: break_reminder_shown
      properties:
        - continuous_use_minutes
        - fatigue_level

    - name: comfort_setting_changed
      properties:
        - setting_type
        - old_value
        - new_value
```

### 5.3 Privacy-Safe Analytics

```typescript
interface PrivacyConfig {
  // 익명화 설정
  anonymization: {
    userId: 'hash';              // SHA-256 해시
    deviceId: 'hash';
    ipAddress: 'drop';           // 수집 안함
    location: 'country_only';
  };

  // 민감 데이터 처리
  sensitiveData: {
    healthMetrics: 'aggregate_only';  // 개별 값 저장 안함
    disabilityDetails: 'category_only'; // 상세 정보 제외
    profileContent: 'never_log';
  };

  // 동의 관리
  consent: {
    required: true;
    granular: true;
    categories: {
      essential: 'always';
      analytics: 'opt_in';
      improvement: 'opt_in';
    };
  };

  // 데이터 보존
  retention: {
    rawEvents: '90_days';
    aggregatedMetrics: '2_years';
    personalData: 'until_deletion_request';
  };
}
```

---

## 6. Alerting System

### 6.1 Alert Rules

```yaml
alert_rules:
  critical:
    - name: high_error_rate
      condition: error_rate > 5%
      for: 5m
      severity: critical
      notification:
        - pagerduty
        - slack_critical

    - name: service_down
      condition: health_check_failures > 3
      for: 1m
      severity: critical
      notification:
        - pagerduty
        - phone_call

    - name: wia_device_safety_triggered
      condition: wia_safety_stop_count > 0
      for: 0m
      severity: critical
      notification:
        - pagerduty
        - email_safety_team

  high:
    - name: high_latency
      condition: p99_latency > 100ms
      for: 10m
      severity: high
      notification:
        - slack_alerts
        - email

    - name: low_frame_rate
      condition: avg_frame_rate < 72
      for: 5m
      severity: high
      notification:
        - slack_alerts

    - name: wia_device_connection_failures
      condition: connection_failure_rate > 10%
      for: 15m
      severity: high
      notification:
        - slack_alerts
        - email

  medium:
    - name: adaptation_fallback_rate_high
      condition: fallback_rate > 5%
      for: 30m
      severity: medium
      notification:
        - slack_alerts

    - name: session_fatigue_high
      condition: avg_fatigue_score > 70
      for: 1h
      severity: medium
      notification:
        - email

  low:
    - name: unusual_usage_pattern
      condition: usage_anomaly_detected
      for: 1h
      severity: low
      notification:
        - email_weekly_digest
```

### 6.2 Alert Configuration

```yaml
alert_configuration:
  routing:
    critical:
      escalation:
        - level_1: on_call_engineer (immediate)
        - level_2: team_lead (after 15m)
        - level_3: engineering_manager (after 30m)

    high:
      escalation:
        - level_1: slack_channel (immediate)
        - level_2: on_call_engineer (after 30m)

  suppression:
    maintenance_windows:
      - schedule: "every sunday 2am-4am UTC"
        suppress: [medium, low]

    deduplication:
      window: 1h
      group_by: [alert_name, service]

  notification_channels:
    pagerduty:
      service_key: "${PAGERDUTY_KEY}"
      severity_mapping:
        critical: high
        high: high
        medium: low

    slack:
      webhook_url: "${SLACK_WEBHOOK}"
      channels:
        critical: "#xr-alerts-critical"
        high: "#xr-alerts"
        medium: "#xr-monitoring"

    email:
      smtp_server: "smtp.wia.org"
      recipients:
        critical: "oncall@wia.org"
        high: "xr-team@wia.org"
        medium: "xr-team@wia.org"
```

---

## 7. Reporting

### 7.1 Standard Reports

```yaml
standard_reports:
  daily:
    - name: daily_health_summary
      schedule: "0 8 * * *"
      recipients: [engineering_team]
      metrics:
        - error_rate_24h
        - p99_latency_24h
        - active_users_24h
        - wia_device_connections_24h

  weekly:
    - name: accessibility_usage_report
      schedule: "0 9 * * MON"
      recipients: [product_team, accessibility_team]
      metrics:
        - feature_usage_by_type
        - disability_category_distribution
        - adaptation_success_rate
        - user_satisfaction_scores

    - name: wia_integration_report
      schedule: "0 9 * * MON"
      recipients: [wia_team]
      metrics:
        - device_connection_stats
        - sync_success_rate
        - latency_percentiles
        - error_breakdown

  monthly:
    - name: accessibility_compliance_report
      schedule: "0 9 1 * *"
      recipients: [compliance_team, leadership]
      metrics:
        - wcag_compliance_score
        - certification_level_distribution
        - user_testing_results
        - improvement_recommendations

    - name: business_metrics_report
      schedule: "0 9 1 * *"
      recipients: [leadership, product]
      metrics:
        - total_users
        - feature_adoption_rates
        - platform_distribution
        - growth_trends
```

### 7.2 Custom Report Builder

```typescript
interface ReportBuilder {
  // 리포트 정의
  definition: {
    name: string;
    description: string;
    schedule?: CronExpression;
    recipients: string[];
  };

  // 데이터 소스
  dataSources: {
    metrics: MetricQuery[];
    events: EventQuery[];
    logs: LogQuery[];
  };

  // 시각화
  visualizations: {
    charts: ChartConfig[];
    tables: TableConfig[];
    summaries: SummaryConfig[];
  };

  // 출력 형식
  output: {
    formats: ['pdf', 'html', 'csv'];
    delivery: ['email', 'slack', 'storage'];
  };
}

// 예시: 접근성 기능 사용 리포트
const accessibilityUsageReport: ReportBuilder = {
  definition: {
    name: "Accessibility Feature Usage",
    description: "Weekly breakdown of accessibility feature usage",
    schedule: "0 9 * * MON",
    recipients: ["accessibility-team@wia.org"],
  },
  dataSources: {
    metrics: [
      { name: "xr_accessibility_feature_usage", aggregation: "sum", groupBy: ["feature_type"] },
      { name: "xr_adaptation_success_rate", aggregation: "avg" },
    ],
    events: [
      { name: "adaptation_applied", filters: { success: true } },
    ],
  },
  visualizations: {
    charts: [
      { type: "pie", title: "Feature Usage Distribution", metric: "feature_usage" },
      { type: "line", title: "Usage Trend", metric: "daily_usage", timeRange: "7d" },
    ],
    tables: [
      { title: "Top Features", columns: ["feature", "usage_count", "success_rate"] },
    ],
  },
  output: {
    formats: ["pdf", "html"],
    delivery: ["email"],
  },
};
```

---

## 8. Data Retention & Privacy

### 8.1 Retention Policies

```yaml
data_retention:
  metrics:
    raw:
      retention: 30_days
      resolution: 15_seconds

    hourly_aggregated:
      retention: 90_days
      resolution: 1_hour

    daily_aggregated:
      retention: 2_years
      resolution: 1_day

  logs:
    error:
      retention: 90_days
      storage: hot

    warn:
      retention: 30_days
      storage: warm

    info:
      retention: 14_days
      storage: warm

    debug:
      retention: 7_days
      storage: cold

  events:
    raw:
      retention: 90_days
      anonymization: immediate

    aggregated:
      retention: 2_years
      pii_removed: true

  user_data:
    profiles:
      retention: until_deletion
      backup: 90_days

    session_data:
      retention: 30_days
      anonymized_summary: 2_years
```

### 8.2 Privacy Compliance

```yaml
privacy_compliance:
  gdpr:
    data_subject_rights:
      - access: supported
      - rectification: supported
      - erasure: supported
      - portability: supported
      - restriction: supported

    legal_basis: consent

    dpa_notifications:
      breach_timeline: 72_hours

  data_processing:
    purposes:
      - service_delivery
      - improvement
      - safety_monitoring
      - compliance

    minimization:
      collect_only_necessary: true
      anonymize_where_possible: true
      aggregate_for_analytics: true

  cross_border:
    data_residency: user_region_preferred
    transfer_mechanisms:
      - standard_contractual_clauses
      - adequacy_decisions
```

---

## 9. Implementation

### 9.1 SDK Integration

```rust
// Rust SDK - Metrics integration
use wia_xr_accessibility::telemetry::*;

async fn setup_telemetry(config: &TelemetryConfig) -> Result<TelemetryClient> {
    let client = TelemetryClient::builder()
        .endpoint(&config.endpoint)
        .api_key(&config.api_key)
        .enable_metrics(true)
        .enable_events(true)
        .enable_logs(config.logging_enabled)
        .privacy_mode(PrivacyMode::Strict)
        .build()
        .await?;

    // 자동 메트릭 수집 시작
    client.start_auto_collection().await?;

    Ok(client)
}

// 커스텀 이벤트 전송
async fn track_accessibility_event(
    client: &TelemetryClient,
    event_name: &str,
    properties: HashMap<String, Value>,
) {
    let event = AnalyticsEvent::builder()
        .name(event_name)
        .properties(properties)
        .accessibility_context(get_current_context())
        .build();

    client.track(event).await;
}
```

### 9.2 Dashboard Configuration

```yaml
# Grafana dashboard configuration
dashboards:
  xr_accessibility_overview:
    title: "WIA XR Accessibility Overview"
    refresh: 30s

    rows:
      - title: "System Health"
        panels:
          - type: stat
            title: "Active Users"
            query: sum(xr_active_sessions)

          - type: gauge
            title: "Avg Frame Rate"
            query: avg(xr_frame_rate_fps)
            thresholds: [60, 72, 90]

          - type: graph
            title: "Error Rate"
            query: rate(xr_errors_total[5m])

      - title: "Accessibility Features"
        panels:
          - type: piechart
            title: "Feature Usage"
            query: sum by(feature_type)(xr_accessibility_feature_usage)

          - type: stat
            title: "Caption Accuracy"
            query: avg(xr_caption_sync_accuracy_percent)

      - title: "WIA Devices"
        panels:
          - type: stat
            title: "Connected Devices"
            query: count(wia_device_connection_status == 2)

          - type: graph
            title: "Sync Latency"
            query: histogram_quantile(0.99, wia_sync_latency_ms)
```

---

## 10. References

- OpenTelemetry Specification
- Prometheus Best Practices
- GDPR Technical Measures
- WIA Analytics Framework v1.0
