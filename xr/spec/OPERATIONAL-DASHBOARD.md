# WIA XR Accessibility: Operational Dashboard Specification

## 1. Overview

본 문서는 XR 접근성 시스템의 운영 대시보드 표준을 정의합니다.

**Version:** 1.0.0
**Status:** Draft
**Phase:** 4 - Integration & Deployment Protocol

---

## 2. Dashboard Architecture

### 2.1 Dashboard Hierarchy

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    WIA XR Dashboard Hierarchy                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────┐        │
│  │                    Executive Dashboard                       │        │
│  │  • KPI 요약                                                  │        │
│  │  • 비즈니스 메트릭                                            │        │
│  │  • 접근성 준수 현황                                           │        │
│  └─────────────────────────────────────────────────────────────┘        │
│                              │                                          │
│         ┌────────────────────┼────────────────────┐                      │
│         │                    │                    │                      │
│  ┌──────▼──────┐     ┌───────▼──────┐     ┌──────▼──────┐               │
│  │ Operations  │     │ Accessibility │     │   WIA       │               │
│  │  Dashboard  │     │  Dashboard    │     │ Devices     │               │
│  │             │     │               │     │ Dashboard   │               │
│  └──────┬──────┘     └───────┬──────┘     └──────┬──────┘               │
│         │                    │                    │                      │
│         │    ┌───────────────┼───────────────┐    │                      │
│         │    │               │               │    │                      │
│  ┌──────▼────▼──┐    ┌───────▼──────┐   ┌───▼────▼──────┐               │
│  │  Platform    │    │   Feature    │   │   Device      │               │
│  │  Specific    │    │   Specific   │   │   Specific    │               │
│  │  Dashboards  │    │   Dashboards │   │   Dashboards  │               │
│  └──────────────┘    └──────────────┘   └───────────────┘               │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Dashboard Components

```typescript
interface DashboardComponent {
  // 컴포넌트 타입
  type: 'stat' | 'gauge' | 'graph' | 'table' | 'heatmap' | 'map' | 'alert_list';

  // 데이터 소스
  dataSource: {
    type: 'prometheus' | 'elasticsearch' | 'custom_api';
    query: string;
    refreshInterval: number;
  };

  // 시각화 설정
  visualization: {
    title: string;
    description?: string;
    thresholds?: Threshold[];
    colors?: ColorScheme;
    legend?: boolean;
  };

  // 상호작용
  interaction: {
    drilldown?: string;
    filters?: Filter[];
    timeRange?: TimeRangeSelector;
  };
}
```

---

## 3. Executive Dashboard

### 3.1 KPI Summary

```yaml
executive_kpis:
  row_1_health:
    - component: system_health_score
      type: gauge
      title: "System Health"
      query: |
        (1 - (sum(rate(xr_errors_total[5m])) / sum(rate(xr_requests_total[5m])))) * 100
      thresholds:
        - value: 99
          color: green
        - value: 95
          color: yellow
        - value: 0
          color: red

    - component: active_users
      type: stat
      title: "Active Users"
      query: sum(xr_active_sessions)
      trend: 24h

    - component: accessibility_score
      type: gauge
      title: "Accessibility Score"
      query: avg(xr_accessibility_quality_score)
      thresholds:
        - value: 90
          color: green
        - value: 75
          color: yellow
        - value: 0
          color: red

  row_2_performance:
    - component: avg_frame_rate
      type: stat
      title: "Avg Frame Rate"
      query: avg(xr_frame_rate_fps)
      unit: fps
      sparkline: true

    - component: p99_latency
      type: stat
      title: "P99 Latency"
      query: histogram_quantile(0.99, xr_mtp_latency_ms)
      unit: ms
      sparkline: true

    - component: error_rate
      type: stat
      title: "Error Rate"
      query: |
        sum(rate(xr_errors_total[5m])) / sum(rate(xr_requests_total[5m])) * 100
      unit: "%"
      sparkline: true

  row_3_wia:
    - component: wia_devices_connected
      type: stat
      title: "WIA Devices"
      query: count(wia_device_connection_status == 2)
      breakdown:
        - exoskeleton
        - bionic_eye
        - voice_sign

    - component: wia_sync_success
      type: gauge
      title: "WIA Sync Success"
      query: avg(wia_sync_success_rate) * 100
      unit: "%"
```

### 3.2 Business Metrics

```yaml
business_metrics:
  user_growth:
    type: graph
    title: "User Growth Trend"
    queries:
      - name: total_users
        query: sum(xr_total_users)
      - name: new_users
        query: sum(increase(xr_new_users_total[24h]))
    timeRange: 30d

  feature_adoption:
    type: bar_chart
    title: "Feature Adoption Rate"
    query: |
      sum by(feature)(xr_accessibility_feature_usage) /
      sum(xr_total_sessions) * 100
    labels:
      - captions
      - audio_description
      - screen_reader
      - voice_control
      - eye_tracking

  certification_distribution:
    type: pie_chart
    title: "Certification Level Distribution"
    query: sum by(level)(xr_certification_level)
    labels:
      - bronze
      - silver
      - gold
      - platinum
```

---

## 4. Operations Dashboard

### 4.1 System Health

```yaml
system_health_dashboard:
  row_1_overview:
    - component: service_status
      type: status_grid
      title: "Service Status"
      services:
        - name: profile_service
          query: up{job="profile-service"}
        - name: adaptation_service
          query: up{job="adaptation-service"}
        - name: wia_gateway
          query: up{job="wia-gateway"}
        - name: analytics_service
          query: up{job="analytics-service"}

    - component: error_breakdown
      type: bar_chart
      title: "Errors by Type"
      query: sum by(error_type)(rate(xr_errors_total[1h]))

  row_2_performance:
    - component: request_rate
      type: graph
      title: "Request Rate"
      queries:
        - name: success
          query: sum(rate(xr_requests_total{status="success"}[5m]))
        - name: error
          query: sum(rate(xr_requests_total{status="error"}[5m]))

    - component: latency_histogram
      type: heatmap
      title: "Latency Distribution"
      query: sum(rate(xr_request_duration_seconds_bucket[5m])) by(le)

  row_3_resources:
    - component: cpu_usage
      type: graph
      title: "CPU Usage"
      query: avg by(instance)(rate(process_cpu_seconds_total[5m]))

    - component: memory_usage
      type: graph
      title: "Memory Usage"
      query: process_resident_memory_bytes / 1024 / 1024

    - component: active_connections
      type: stat
      title: "Active Connections"
      query: sum(xr_active_connections)
```

### 4.2 Platform-Specific Views

```yaml
platform_dashboards:
  quest:
    title: "Meta Quest Platform"
    panels:
      - frame_rate_quest:
          query: avg(xr_frame_rate_fps{platform="quest"})

      - quest_specific_errors:
          query: sum by(error)(rate(xr_errors_total{platform="quest"}[5m]))

      - quest_users:
          query: sum(xr_active_sessions{platform="quest"})

  vision_pro:
    title: "Apple Vision Pro Platform"
    panels:
      - frame_rate_vision:
          query: avg(xr_frame_rate_fps{platform="vision_pro"})

      - vision_specific_features:
          query: sum by(feature)(xr_accessibility_feature_usage{platform="vision_pro"})

  psvr:
    title: "PlayStation VR Platform"
    panels:
      - frame_rate_psvr:
          query: avg(xr_frame_rate_fps{platform="psvr"})

      - psvr_users:
          query: sum(xr_active_sessions{platform="psvr"})
```

---

## 5. Accessibility Dashboard

### 5.1 Feature Usage

```yaml
accessibility_features_dashboard:
  row_1_overview:
    - component: feature_usage_pie
      type: pie_chart
      title: "Feature Usage Distribution"
      query: sum by(feature_type)(xr_accessibility_feature_usage)

    - component: disability_distribution
      type: pie_chart
      title: "User Disability Categories"
      query: sum by(category)(xr_user_disability_category)
      note: "Anonymized aggregate data only"

  row_2_visual:
    - component: caption_usage
      type: graph
      title: "Caption Usage Over Time"
      queries:
        - name: enabled
          query: sum(xr_accessibility_feature_usage{feature="captions", enabled="true"})
        - name: accuracy
          query: avg(xr_caption_sync_accuracy_percent)

    - component: screen_reader_activity
      type: graph
      title: "Screen Reader Activity"
      query: sum(rate(xr_screen_reader_interactions_total[5m]))

    - component: audio_description
      type: stat
      title: "Audio Descriptions Played"
      query: sum(increase(xr_audio_description_played_count[24h]))

  row_3_input:
    - component: input_method_usage
      type: bar_chart
      title: "Input Method Usage"
      query: sum by(method)(xr_input_method_usage)
      labels:
        - voice_control
        - eye_tracking
        - hand_tracking
        - switch_access
        - controller

    - component: voice_command_success
      type: gauge
      title: "Voice Command Success Rate"
      query: |
        sum(xr_input_method_usage{method="voice_control", success="true"}) /
        sum(xr_input_method_usage{method="voice_control"}) * 100

  row_4_safety:
    - component: session_health
      type: graph
      title: "Average Session Health"
      queries:
        - name: fatigue_score
          query: avg(xr_session_fatigue_score)
        - name: break_compliance
          query: sum(xr_break_taken_count) / sum(xr_break_reminder_triggered_count) * 100

    - component: emergency_exits
      type: stat
      title: "Emergency Exits (24h)"
      query: sum(increase(xr_emergency_exit_triggered[24h]))
      alert_threshold: 10
```

### 5.2 Quality Metrics

```yaml
accessibility_quality_dashboard:
  row_1_scores:
    - component: overall_quality_score
      type: gauge
      title: "Overall Accessibility Quality"
      query: avg(xr_accessibility_quality_score)
      thresholds:
        - value: 90
          label: "Excellent"
          color: green
        - value: 75
          label: "Good"
          color: yellow
        - value: 60
          label: "Acceptable"
          color: orange
        - value: 0
          label: "Needs Work"
          color: red

    - component: category_scores
      type: radar_chart
      title: "Quality by Category"
      metrics:
        - visual_accessibility
        - auditory_accessibility
        - motor_accessibility
        - cognitive_accessibility
        - comfort_safety

  row_2_trends:
    - component: quality_trend
      type: graph
      title: "Quality Score Trend"
      query: avg(xr_accessibility_quality_score)
      timeRange: 90d

    - component: improvement_areas
      type: table
      title: "Improvement Opportunities"
      query: |
        bottomk(5, avg by(feature)(xr_accessibility_quality_score))
      columns:
        - feature
        - score
        - trend

  row_3_compliance:
    - component: wcag_compliance
      type: table
      title: "WCAG Compliance Status"
      data:
        - criterion: "1.1 Text Alternatives"
          status: passed
          score: 100
        - criterion: "1.2 Time-based Media"
          status: passed
          score: 95
        - criterion: "1.3 Adaptable"
          status: warning
          score: 85
        # ... more criteria

    - component: certification_status
      type: status_indicator
      title: "WIA Certification Status"
      query: xr_certification_level
      mapping:
        0: "Not Certified"
        1: "Bronze"
        2: "Silver"
        3: "Gold"
        4: "Platinum"
```

---

## 6. WIA Devices Dashboard

### 6.1 Device Overview

```yaml
wia_devices_overview:
  row_1_status:
    - component: device_summary
      type: stat_grid
      title: "WIA Device Summary"
      items:
        - name: "Exoskeletons"
          query: count(wia_device_connection_status{type="exoskeleton"} == 2)
          icon: exoskeleton

        - name: "Bionic Eyes"
          query: count(wia_device_connection_status{type="bionic_eye"} == 2)
          icon: eye

        - name: "Voice-Sign"
          query: count(wia_device_connection_status{type="voice_sign"} == 2)
          icon: sign_language

    - component: connection_status
      type: table
      title: "Device Connection Status"
      query: wia_device_connection_status
      columns:
        - device_id
        - device_type
        - status
        - last_seen
        - battery

  row_2_health:
    - component: sync_latency
      type: graph
      title: "Sync Latency by Device Type"
      queries:
        - name: exoskeleton
          query: histogram_quantile(0.99, wia_sync_latency_ms{type="exoskeleton"})
        - name: bionic_eye
          query: histogram_quantile(0.99, wia_sync_latency_ms{type="bionic_eye"})
        - name: voice_sign
          query: histogram_quantile(0.99, wia_sync_latency_ms{type="voice_sign"})

    - component: sync_success_rate
      type: graph
      title: "Sync Success Rate"
      query: avg by(type)(wia_sync_success_rate) * 100

  row_3_errors:
    - component: connection_errors
      type: graph
      title: "Connection Errors"
      query: sum by(type)(rate(wia_device_connection_errors_total[5m]))

    - component: error_breakdown
      type: table
      title: "Error Breakdown"
      query: topk(10, sum by(type, error)(wia_device_errors_total))
```

### 6.2 Device-Specific Dashboards

```yaml
exoskeleton_dashboard:
  title: "WIA Exoskeleton Dashboard"

  row_1_haptics:
    - component: haptic_commands
      type: graph
      title: "Haptic Commands Sent"
      query: sum(rate(wia_exoskeleton_haptic_commands[5m]))

    - component: haptic_latency
      type: histogram
      title: "Haptic Command Latency"
      query: wia_exoskeleton_haptic_latency_ms

  row_2_motion:
    - component: motion_assistance_usage
      type: gauge
      title: "Motion Assistance Active"
      query: |
        sum(wia_exoskeleton_assistance_active) /
        sum(wia_exoskeleton_connected) * 100

    - component: assistance_levels
      type: bar_chart
      title: "Assistance Level Distribution"
      query: sum by(level)(wia_exoskeleton_assistance_level)

  row_3_safety:
    - component: safety_stops
      type: stat
      title: "Safety Stops (24h)"
      query: sum(increase(wia_exoskeleton_safety_stop_count[24h]))
      alert_threshold: 1

    - component: force_limits
      type: graph
      title: "Force vs Limits"
      queries:
        - name: actual_force
          query: avg(wia_exoskeleton_force_newtons)
        - name: limit
          query: avg(wia_exoskeleton_force_limit_newtons)

bionic_eye_dashboard:
  title: "WIA Bionic Eye Dashboard"

  row_1_stimulation:
    - component: stimulation_time
      type: graph
      title: "Stimulation vs Rest Time"
      queries:
        - name: stimulation
          query: sum(rate(wia_bionic_eye_stimulation_time_seconds[5m]))
        - name: rest
          query: sum(rate(wia_bionic_eye_rest_time_seconds[5m]))

    - component: brightness_levels
      type: histogram
      title: "Brightness Level Distribution"
      query: wia_bionic_eye_brightness_level

  row_2_safety:
    - component: duty_ratio
      type: gauge
      title: "Stimulation Duty Ratio"
      query: |
        sum(wia_bionic_eye_stimulation_time_seconds) /
        (sum(wia_bionic_eye_stimulation_time_seconds) +
         sum(wia_bionic_eye_rest_time_seconds)) * 100
      thresholds:
        - value: 50
          color: green
        - value: 70
          color: yellow
        - value: 80
          color: red

voice_sign_dashboard:
  title: "WIA Voice-Sign Dashboard"

  row_1_translation:
    - component: translations_count
      type: stat
      title: "Translations (24h)"
      query: sum(increase(wia_voice_sign_translations_total[24h]))

    - component: translation_latency
      type: histogram
      title: "Translation Latency"
      query: wia_voice_sign_translation_latency_ms

  row_2_quality:
    - component: translation_confidence
      type: gauge
      title: "Average Confidence"
      query: avg(wia_voice_sign_translation_confidence) * 100

    - component: fallback_rate
      type: stat
      title: "Fallback Rate"
      query: |
        sum(wia_voice_sign_fallback_used) /
        sum(wia_voice_sign_translations_total) * 100
```

---

## 7. Alert Dashboard

### 7.1 Active Alerts

```yaml
alert_dashboard:
  row_1_summary:
    - component: alert_summary
      type: stat_grid
      title: "Alert Summary"
      items:
        - name: "Critical"
          query: count(ALERTS{severity="critical", alertstate="firing"})
          color: red

        - name: "High"
          query: count(ALERTS{severity="high", alertstate="firing"})
          color: orange

        - name: "Medium"
          query: count(ALERTS{severity="medium", alertstate="firing"})
          color: yellow

        - name: "Low"
          query: count(ALERTS{severity="low", alertstate="firing"})
          color: blue

  row_2_active:
    - component: active_alerts_list
      type: alert_table
      title: "Active Alerts"
      columns:
        - severity
        - alert_name
        - service
        - message
        - duration
        - actions
      filters:
        - alertstate: firing
      sort: severity_desc

  row_3_history:
    - component: alert_timeline
      type: timeline
      title: "Alert Timeline (24h)"
      query: ALERTS{alertstate="firing"}
      timeRange: 24h

    - component: alert_frequency
      type: bar_chart
      title: "Alert Frequency by Type"
      query: count by(alertname)(ALERTS)
      timeRange: 7d
```

### 7.2 Incident Management

```yaml
incident_dashboard:
  row_1_active_incidents:
    - component: incident_list
      type: table
      title: "Active Incidents"
      columns:
        - incident_id
        - severity
        - title
        - status
        - assigned_to
        - duration
        - affected_users
      actions:
        - acknowledge
        - escalate
        - resolve

  row_2_metrics:
    - component: mttr
      type: stat
      title: "MTTR (Mean Time to Resolve)"
      query: avg(incident_resolution_time_minutes)
      trend: 30d

    - component: mtta
      type: stat
      title: "MTTA (Mean Time to Acknowledge)"
      query: avg(incident_acknowledge_time_minutes)
      trend: 30d

    - component: incident_count
      type: graph
      title: "Incident Count Trend"
      query: count(incidents_total)
      timeRange: 30d
```

---

## 8. User-Facing Dashboard

### 8.1 Accessibility Status Display

```yaml
user_accessibility_dashboard:
  description: "사용자에게 표시되는 접근성 상태"

  components:
    profile_status:
      type: status_card
      title: "내 접근성 프로필"
      items:
        - "활성 기능: 3개"
        - "마지막 동기화: 5분 전"
        - "WIA 기기: 1대 연결됨"

    session_health:
      type: health_indicator
      title: "세션 건강"
      metrics:
        - fatigue_level
        - eye_strain_estimate
        - session_duration
      recommendations: true

    wia_devices:
      type: device_status
      title: "WIA 기기 상태"
      devices:
        - type: exoskeleton
          status: connected
          battery: 85%
        - type: voice_sign
          status: connected
          quality: high

    quick_settings:
      type: toggle_grid
      title: "빠른 설정"
      settings:
        - caption_toggle
        - audio_description_toggle
        - safe_space_button
        - rest_reminder_toggle
```

### 8.2 Accessibility Preferences Panel

```yaml
user_preferences_panel:
  sections:
    visual:
      title: "시각 설정"
      controls:
        - type: toggle
          name: captions
          label: "자막"

        - type: slider
          name: font_size
          label: "글꼴 크기"
          range: [12, 48]

        - type: color_picker
          name: caption_color
          label: "자막 색상"

    auditory:
      title: "청각 설정"
      controls:
        - type: toggle
          name: audio_descriptions
          label: "오디오 설명"

        - type: select
          name: sign_language
          label: "수화 언어"
          options: [ASL, BSL, KSL, JSL]

    comfort:
      title: "편의 설정"
      controls:
        - type: toggle
          name: motion_comfort
          label: "모션 편의"

        - type: slider
          name: vignette_intensity
          label: "비네트 강도"
          range: [0, 100]

        - type: number
          name: break_interval
          label: "휴식 알림 간격"
          unit: "분"
```

---

## 9. Dashboard Access Control

### 9.1 Role-Based Access

```yaml
dashboard_access:
  roles:
    admin:
      dashboards: all
      actions:
        - view
        - edit
        - create
        - delete
        - share

    operator:
      dashboards:
        - operations
        - accessibility
        - wia_devices
        - alerts
      actions:
        - view
        - acknowledge_alerts

    analyst:
      dashboards:
        - executive
        - accessibility
        - analytics
      actions:
        - view
        - export

    developer:
      dashboards:
        - operations
        - platform_specific
      actions:
        - view

    user:
      dashboards:
        - user_accessibility
        - user_preferences
      actions:
        - view
        - update_own_settings
```

### 9.2 Data Visibility

```yaml
data_visibility:
  pii_handling:
    user_ids: anonymized
    device_ids: hashed
    health_data: aggregated_only

  sensitive_metrics:
    disability_info:
      visibility: admin_only
      display: aggregated_counts

    location_data:
      visibility: none
      collection: disabled

  audit_logging:
    dashboard_access: true
    data_exports: true
    setting_changes: true
```

---

## 10. Implementation

### 10.1 Grafana Configuration

```yaml
# grafana/provisioning/dashboards/wia-xr.yaml
apiVersion: 1

providers:
  - name: 'WIA XR Dashboards'
    orgId: 1
    folder: 'WIA XR Accessibility'
    folderUid: 'wia-xr'
    type: file
    disableDeletion: false
    editable: true
    options:
      path: /var/lib/grafana/dashboards/wia-xr

# grafana/provisioning/datasources/prometheus.yaml
apiVersion: 1

datasources:
  - name: Prometheus
    type: prometheus
    access: proxy
    url: http://prometheus:9090
    isDefault: true

  - name: Loki
    type: loki
    access: proxy
    url: http://loki:3100

  - name: Elasticsearch
    type: elasticsearch
    access: proxy
    url: http://elasticsearch:9200
    database: "wia-xr-logs-*"
```

### 10.2 Dashboard JSON Template

```json
{
  "dashboard": {
    "id": null,
    "uid": "wia-xr-overview",
    "title": "WIA XR Accessibility Overview",
    "tags": ["wia", "xr", "accessibility"],
    "timezone": "browser",
    "refresh": "30s",
    "panels": [
      {
        "id": 1,
        "type": "gauge",
        "title": "System Health",
        "gridPos": { "h": 8, "w": 6, "x": 0, "y": 0 },
        "targets": [
          {
            "expr": "(1 - (sum(rate(xr_errors_total[5m])) / sum(rate(xr_requests_total[5m])))) * 100",
            "refId": "A"
          }
        ],
        "fieldConfig": {
          "defaults": {
            "thresholds": {
              "steps": [
                { "value": 0, "color": "red" },
                { "value": 95, "color": "yellow" },
                { "value": 99, "color": "green" }
              ]
            },
            "unit": "percent"
          }
        }
      }
    ]
  }
}
```

---

## 11. References

- Grafana Documentation
- Prometheus Alertmanager
- OpenTelemetry Visualization
- WIA Dashboard Standards v1.0
