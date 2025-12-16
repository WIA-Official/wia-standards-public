# WIA Voice-Sign Operational Dashboard Specification

Version: 1.0.0
Status: Draft
Last Updated: 2025-01-15

## 1. Overview

This specification defines the operational dashboard for administrators and operators of WIA Voice-Sign translation systems.

## 2. Dashboard Architecture

### 2.1 System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     Dashboard Frontend                          │
│                    (React / Vue.js)                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐            │
│  │   Auth      │  │   API       │  │  WebSocket  │            │
│  │  Module     │  │  Client     │  │   Client    │            │
│  └─────────────┘  └─────────────┘  └─────────────┘            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Dashboard Backend                            │
│                    (API Gateway)                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐            │
│  │  Metrics    │  │   Config    │  │   Audit     │            │
│  │  Service    │  │   Service   │  │   Service   │            │
│  └─────────────┘  └─────────────┘  └─────────────┘            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Data Sources                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐            │
│  │ Prometheus  │  │  Database   │  │   Logs      │            │
│  │             │  │ (PostgreSQL)│  │(Elasticsearch)            │
│  └─────────────┘  └─────────────┘  └─────────────┘            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Technology Stack

```yaml
frontend:
  framework: React 18+
  state_management: Redux Toolkit
  ui_library: Ant Design / Material UI
  charting: Apache ECharts / Recharts
  real_time: Socket.io

backend:
  api: REST + GraphQL
  real_time: WebSocket
  authentication: OAuth 2.0 / OIDC

data_sources:
  metrics: Prometheus + Grafana
  logs: Elasticsearch + Kibana
  database: PostgreSQL
  cache: Redis
```

## 3. Admin Dashboard Features

### 3.1 Overview Page

```yaml
overview_page:
  sections:
    - title: "System Status"
      widgets:
        - type: status_indicator
          services:
            - voice_sign_api
            - asr_service
            - translation_service
            - renderer

        - type: stat_card
          metrics:
            - name: "Active Users"
              value: real_time
            - name: "Translations Today"
              value: counter
            - name: "Avg Response Time"
              value: gauge
            - name: "Error Rate"
              value: percentage

    - title: "Real-time Activity"
      widgets:
        - type: live_chart
          metric: requests_per_second
          window: 5m

        - type: live_map
          data: requests_by_region

    - title: "Alerts"
      widgets:
        - type: alert_list
          filter: active
          severity: [critical, warning]
```

### 3.2 Monitoring Views

```yaml
monitoring_views:
  - name: "Service Health"
    panels:
      - type: service_grid
        services:
          - name: API Gateway
            health_endpoint: /health
            metrics: [cpu, memory, requests]

          - name: ASR Service
            health_endpoint: /health
            metrics: [cpu, memory, gpu, queue_depth]

          - name: Translation Service
            health_endpoint: /health
            metrics: [cpu, memory, cache_hit_rate]

          - name: Renderer
            health_endpoint: /health
            metrics: [cpu, memory, gpu, render_queue]

  - name: "Performance"
    panels:
      - type: latency_histogram
        breakdown: [p50, p90, p95, p99]

      - type: throughput_chart
        metrics: [requests, translations, renders]

      - type: error_rate_chart
        breakdown: by_endpoint

  - name: "Quality Metrics"
    panels:
      - type: quality_score_trend
        window: 24h
        breakdown: by_language

      - type: confidence_distribution
        histogram: true

      - type: fallback_rate
        breakdown: by_reason
```

### 3.3 User Management

```yaml
user_management:
  features:
    - user_list
    - user_search
    - user_details
    - role_assignment
    - api_key_management
    - usage_quotas

  user_list:
    columns:
      - id
      - email (masked)
      - role
      - status
      - last_active
      - usage_quota
      - actions

    filters:
      - role
      - status
      - registration_date
      - usage_level

    actions:
      - view_details
      - edit_role
      - suspend
      - delete
      - reset_api_key

  roles:
    - name: admin
      permissions: [all]

    - name: operator
      permissions:
        - view_monitoring
        - manage_config
        - view_users

    - name: analyst
      permissions:
        - view_monitoring
        - view_analytics
        - export_data

    - name: support
      permissions:
        - view_users
        - view_logs
        - create_tickets
```

### 3.4 Configuration Management

```yaml
configuration_management:
  sections:
    - name: "System Configuration"
      settings:
        - key: rate_limit.default
          type: number
          description: "Default rate limit per minute"
          validation: min=10, max=10000

        - key: quality.min_threshold
          type: number
          description: "Minimum quality score threshold"
          validation: min=0, max=1

        - key: safety.content_filter_enabled
          type: boolean
          description: "Enable content filtering"

        - key: safety.emergency_detection_enabled
          type: boolean
          description: "Enable emergency detection"

    - name: "Model Configuration"
      settings:
        - key: asr.model_version
          type: select
          options: [whisper-v2, whisper-v3, custom-v1]

        - key: translation.cache_ttl
          type: number
          description: "Translation cache TTL in seconds"

    - name: "Feature Flags"
      settings:
        - key: features.streaming_enabled
          type: boolean

        - key: features.batch_mode_enabled
          type: boolean

        - key: features.experimental_avatar
          type: boolean
          rollout_percentage: 10

  audit:
    enabled: true
    fields:
      - timestamp
      - user
      - config_key
      - old_value
      - new_value
      - reason

  rollback:
    enabled: true
    history_retention: 30_days
```

## 4. Real-time Monitoring Views

### 4.1 Live Dashboard

```typescript
interface LiveDashboardConfig {
  refresh_interval_ms: 1000;

  panels: {
    // Request stream
    request_stream: {
      type: "live_table";
      columns: ["time", "request_id", "endpoint", "status", "latency"];
      max_rows: 100;
      highlight_errors: true;
    };

    // Current metrics
    current_metrics: {
      type: "stat_grid";
      metrics: [
        { name: "RPS", source: "prometheus", query: "rate(requests_total[1m])" },
        { name: "P99 Latency", source: "prometheus", query: "histogram_quantile(0.99, ...)" },
        { name: "Active Connections", source: "prometheus", query: "active_connections" },
        { name: "Error Rate", source: "prometheus", query: "rate(errors_total[1m])" }
      ];
    };

    // Real-time charts
    realtime_charts: {
      type: "streaming_chart";
      charts: [
        { metric: "requests_per_second", window: "5m" },
        { metric: "latency_p99", window: "5m" },
        { metric: "error_rate", window: "5m" }
      ];
    };
  };
}
```

### 4.2 Alert Management

```yaml
alert_management:
  views:
    - name: "Active Alerts"
      columns:
        - severity
        - alert_name
        - message
        - source
        - started_at
        - duration
        - actions

      actions:
        - acknowledge
        - silence (duration)
        - escalate
        - resolve

    - name: "Alert History"
      filters:
        - date_range
        - severity
        - source
        - status

    - name: "Alert Rules"
      features:
        - view_rules
        - edit_rules
        - enable_disable
        - test_rule

  notifications:
    channels:
      - slack
      - email
      - pagerduty
      - webhook

    routing:
      critical: [pagerduty, slack]
      warning: [slack]
      info: [email]

  silencing:
    options:
      - duration: [15m, 1h, 4h, 24h, custom]
      - scope: [specific_alert, alert_group, all]
      - require_reason: true
```

### 4.3 Log Viewer

```yaml
log_viewer:
  features:
    - search: full_text
    - filter:
        - level: [trace, debug, info, warn, error, fatal]
        - service: [api, asr, translation, renderer]
        - time_range: [15m, 1h, 6h, 24h, 7d, custom]
    - stream: real_time
    - export: [json, csv]

  display:
    columns:
      - timestamp
      - level
      - service
      - message
      - trace_id
      - details (expandable)

    syntax_highlighting: true
    json_formatting: true

  correlation:
    - by_trace_id
    - by_request_id
    - by_session_id

  saved_searches:
    enabled: true
    sharing: team
```

## 5. Audit Logging

### 5.1 Audit Events

```typescript
interface AuditEvent {
  event_id: string;
  timestamp: string;
  event_type: AuditEventType;
  actor: {
    user_id: string;
    username: string;
    role: string;
    ip_address: string;
  };
  resource: {
    type: string;
    id: string;
    name?: string;
  };
  action: string;
  outcome: "success" | "failure";
  details: Record<string, any>;
  metadata: {
    user_agent: string;
    session_id: string;
    request_id: string;
  };
}

type AuditEventType =
  | "authentication"
  | "authorization"
  | "configuration_change"
  | "user_management"
  | "data_access"
  | "system_operation"
  | "security_event";
```

### 5.2 Audit Log Viewer

```yaml
audit_log_viewer:
  filters:
    - time_range
    - event_type
    - actor
    - resource_type
    - outcome

  columns:
    - timestamp
    - event_type
    - actor
    - action
    - resource
    - outcome
    - details (expandable)

  export:
    formats: [json, csv]
    compliance: [SOC2, GDPR]

  retention:
    duration: 7_years
    immutable: true
    encryption: at_rest
```

### 5.3 Compliance Reports

```yaml
compliance_reports:
  types:
    - name: "Access Report"
      description: "Who accessed what and when"
      schedule: daily
      format: pdf

    - name: "Configuration Changes"
      description: "All configuration modifications"
      schedule: weekly
      format: pdf

    - name: "Security Events"
      description: "Authentication failures, suspicious activity"
      schedule: daily
      format: pdf

    - name: "Data Access"
      description: "PII access and handling"
      schedule: monthly
      format: pdf

  distribution:
    email: compliance_team
    storage: secure_archive
```

## 6. Incident Management

### 6.1 Incident Workflow

```yaml
incident_workflow:
  statuses:
    - detected
    - acknowledged
    - investigating
    - identified
    - mitigating
    - resolved
    - post_mortem

  severity_levels:
    - sev1: "Critical - Service down"
    - sev2: "Major - Degraded performance"
    - sev3: "Minor - Limited impact"
    - sev4: "Low - Cosmetic/minor issues"

  response_times:
    sev1:
      acknowledge: 5m
      update: 15m
    sev2:
      acknowledge: 15m
      update: 30m
    sev3:
      acknowledge: 1h
      update: 4h
    sev4:
      acknowledge: 4h
      update: 24h
```

### 6.2 Incident Dashboard

```yaml
incident_dashboard:
  views:
    - name: "Active Incidents"
      columns:
        - severity
        - title
        - status
        - assignee
        - duration
        - impacted_services
        - actions

    - name: "Incident Timeline"
      type: timeline
      events:
        - status_changes
        - comments
        - related_alerts
        - actions_taken

    - name: "Post-Mortem"
      sections:
        - summary
        - timeline
        - root_cause
        - impact
        - action_items
        - lessons_learned

  integrations:
    - pagerduty
    - opsgenie
    - slack
    - jira
```

## 7. API for Dashboard

### 7.1 Dashboard API Endpoints

```yaml
dashboard_api:
  base_path: /api/v1/dashboard

  endpoints:
    # Overview
    GET /overview:
      description: "Get dashboard overview data"
      response: OverviewData

    # Metrics
    GET /metrics:
      description: "Get metrics data"
      params:
        - metrics: string[]
        - start_time: datetime
        - end_time: datetime
        - step: duration
      response: MetricsData

    # Real-time stream
    WS /stream:
      description: "Real-time metrics stream"
      events:
        - metric_update
        - alert
        - incident

    # Configuration
    GET /config:
      description: "Get current configuration"
    PUT /config/{key}:
      description: "Update configuration"
      body: ConfigUpdate

    # Users
    GET /users:
      description: "List users"
      params: pagination, filters
    GET /users/{id}:
      description: "Get user details"
    PUT /users/{id}:
      description: "Update user"

    # Audit logs
    GET /audit:
      description: "Get audit logs"
      params: pagination, filters

    # Incidents
    GET /incidents:
      description: "List incidents"
    POST /incidents:
      description: "Create incident"
    PUT /incidents/{id}:
      description: "Update incident"
```

### 7.2 WebSocket Events

```typescript
// WebSocket connection
const ws = new WebSocket("wss://dashboard.wia.org/api/v1/dashboard/stream");

// Subscribe to channels
ws.send(JSON.stringify({
  type: "subscribe",
  channels: ["metrics", "alerts", "incidents"]
}));

// Event types
interface MetricUpdateEvent {
  type: "metric_update";
  timestamp: string;
  metrics: {
    name: string;
    value: number;
    labels: Record<string, string>;
  }[];
}

interface AlertEvent {
  type: "alert";
  alert: {
    id: string;
    name: string;
    severity: string;
    status: "firing" | "resolved";
    message: string;
  };
}

interface IncidentEvent {
  type: "incident";
  incident: {
    id: string;
    title: string;
    severity: string;
    status: string;
    updated_at: string;
  };
}
```

## 8. Access Control

### 8.1 Role-Based Permissions

```yaml
rbac:
  roles:
    super_admin:
      description: "Full system access"
      permissions: ["*"]

    admin:
      description: "Administrative access"
      permissions:
        - dashboard.view
        - config.view
        - config.edit
        - users.view
        - users.edit
        - audit.view
        - incidents.manage

    operator:
      description: "Operations access"
      permissions:
        - dashboard.view
        - config.view
        - alerts.manage
        - incidents.view
        - incidents.update

    analyst:
      description: "Analytics access"
      permissions:
        - dashboard.view
        - metrics.view
        - analytics.view
        - analytics.export

    viewer:
      description: "Read-only access"
      permissions:
        - dashboard.view
        - metrics.view

  permission_groups:
    dashboard:
      - dashboard.view

    config:
      - config.view
      - config.edit

    users:
      - users.view
      - users.edit
      - users.delete

    audit:
      - audit.view
      - audit.export

    incidents:
      - incidents.view
      - incidents.create
      - incidents.update
      - incidents.manage

    alerts:
      - alerts.view
      - alerts.acknowledge
      - alerts.silence
      - alerts.manage
```

### 8.2 Session Management

```yaml
session_management:
  session:
    timeout: 8h
    idle_timeout: 30m
    max_concurrent: 3

  security:
    mfa_required: true
    ip_whitelist: optional
    device_tracking: true

  features:
    - active_sessions_view
    - force_logout
    - session_history
```

## 9. Customization

### 9.1 Dashboard Customization

```yaml
customization:
  themes:
    - light
    - dark
    - high_contrast

  layouts:
    - default
    - compact
    - wide

  widgets:
    customizable: true
    drag_drop: true
    resize: true

  saved_views:
    personal: true
    shared: true
    default: configurable

  notifications:
    in_app: true
    email: configurable
    push: optional
```

### 9.2 Report Builder

```yaml
report_builder:
  features:
    - drag_drop_widgets
    - custom_queries
    - scheduling
    - export_formats: [pdf, xlsx, csv]
    - sharing

  templates:
    - executive_summary
    - technical_report
    - compliance_report
    - custom

  scheduling:
    frequencies:
      - daily
      - weekly
      - monthly
      - custom_cron

    delivery:
      - email
      - slack
      - s3_upload
```

## 10. Mobile Support

### 10.1 Mobile Dashboard

```yaml
mobile_dashboard:
  platforms:
    - ios
    - android
    - pwa

  features:
    - overview_stats
    - alert_notifications
    - incident_management
    - quick_actions

  notifications:
    push: true
    critical_alerts: immediate
    summary: configurable

  offline:
    cached_views: true
    action_queue: true
```

### 10.2 Responsive Design

```yaml
responsive_design:
  breakpoints:
    mobile: 320px - 767px
    tablet: 768px - 1023px
    desktop: 1024px+

  adaptations:
    mobile:
      - simplified_charts
      - collapsed_navigation
      - swipe_gestures

    tablet:
      - sidebar_collapsible
      - touch_optimized

    desktop:
      - full_features
      - keyboard_shortcuts
```
