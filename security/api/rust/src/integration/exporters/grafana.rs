//! Grafana Dashboard Generator
//!
//! Generates Grafana dashboard JSON for WIA Security metrics visualization.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{ExportResult, ExportError};

/// Grafana dashboard configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GrafanaConfig {
    /// Dashboard title
    pub title: String,
    /// Dashboard UID (auto-generated if not provided)
    pub uid: Option<String>,
    /// Data source name
    pub datasource: String,
    /// Data source type
    pub datasource_type: DatasourceType,
    /// Refresh interval
    pub refresh: String,
    /// Time range
    pub time_from: String,
    pub time_to: String,
    /// Tags
    pub tags: Vec<String>,
}

/// Supported data source types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DatasourceType {
    Elasticsearch,
    Prometheus,
    InfluxDB,
    Loki,
}

impl Default for GrafanaConfig {
    fn default() -> Self {
        Self {
            title: "WIA Security Dashboard".to_string(),
            uid: None,
            datasource: "elasticsearch".to_string(),
            datasource_type: DatasourceType::Elasticsearch,
            refresh: "5m".to_string(),
            time_from: "now-24h".to_string(),
            time_to: "now".to_string(),
            tags: vec!["security".to_string(), "wia".to_string()],
        }
    }
}

/// Grafana dashboard
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GrafanaDashboard {
    pub id: Option<u64>,
    pub uid: String,
    pub title: String,
    pub tags: Vec<String>,
    pub timezone: String,
    #[serde(rename = "schemaVersion")]
    pub schema_version: u32,
    pub version: u32,
    pub refresh: String,
    pub time: GrafanaTimeRange,
    pub panels: Vec<GrafanaPanel>,
    pub templating: GrafanaTemplating,
    pub annotations: GrafanaAnnotations,
}

/// Time range
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GrafanaTimeRange {
    pub from: String,
    pub to: String,
}

/// Panel definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GrafanaPanel {
    pub id: u32,
    #[serde(rename = "type")]
    pub panel_type: String,
    pub title: String,
    pub description: Option<String>,
    #[serde(rename = "gridPos")]
    pub grid_pos: GridPosition,
    pub datasource: PanelDatasource,
    pub targets: Vec<PanelTarget>,
    #[serde(rename = "fieldConfig")]
    pub field_config: FieldConfig,
    pub options: serde_json::Value,
}

/// Grid position
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridPosition {
    pub h: u32,
    pub w: u32,
    pub x: u32,
    pub y: u32,
}

/// Panel datasource
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PanelDatasource {
    #[serde(rename = "type")]
    pub ds_type: String,
    pub uid: String,
}

/// Panel target (query)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PanelTarget {
    #[serde(rename = "refId")]
    pub ref_id: String,
    #[serde(flatten)]
    pub query: serde_json::Value,
}

/// Field configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldConfig {
    pub defaults: FieldDefaults,
    pub overrides: Vec<serde_json::Value>,
}

/// Field defaults
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldDefaults {
    pub color: ColorConfig,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,
    pub thresholds: Thresholds,
    pub mappings: Vec<serde_json::Value>,
}

/// Color configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColorConfig {
    pub mode: String,
}

/// Thresholds
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Thresholds {
    pub mode: String,
    pub steps: Vec<ThresholdStep>,
}

/// Threshold step
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThresholdStep {
    pub color: String,
    pub value: Option<f64>,
}

/// Templating
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GrafanaTemplating {
    pub list: Vec<TemplateVariable>,
}

/// Template variable
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemplateVariable {
    pub name: String,
    pub label: Option<String>,
    #[serde(rename = "type")]
    pub var_type: String,
    pub query: Option<serde_json::Value>,
    pub current: Option<serde_json::Value>,
    pub options: Vec<serde_json::Value>,
    pub multi: bool,
    #[serde(rename = "includeAll")]
    pub include_all: bool,
}

/// Annotations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GrafanaAnnotations {
    pub list: Vec<serde_json::Value>,
}

/// Grafana Dashboard Generator
pub struct GrafanaGenerator {
    config: GrafanaConfig,
}

impl GrafanaGenerator {
    /// Create new Grafana generator
    pub fn new(config: GrafanaConfig) -> Self {
        Self { config }
    }

    /// Generate security overview dashboard
    pub fn generate_security_dashboard(&self) -> GrafanaDashboard {
        let uid = self.config.uid.clone()
            .unwrap_or_else(|| format!("wia-security-{}", uuid::Uuid::new_v4().to_string()[..8].to_string()));

        let ds_type = match self.config.datasource_type {
            DatasourceType::Elasticsearch => "elasticsearch",
            DatasourceType::Prometheus => "prometheus",
            DatasourceType::InfluxDB => "influxdb",
            DatasourceType::Loki => "loki",
        };

        let datasource = PanelDatasource {
            ds_type: ds_type.to_string(),
            uid: self.config.datasource.clone(),
        };

        let mut panels = Vec::new();
        let mut panel_id = 1;
        let mut y_pos = 0;

        // Row 1: Key metrics
        panels.push(self.create_stat_panel(
            panel_id,
            "Total Findings",
            GridPosition { h: 4, w: 6, x: 0, y: y_pos },
            datasource.clone(),
            self.total_findings_query(),
        ));
        panel_id += 1;

        panels.push(self.create_stat_panel(
            panel_id,
            "Critical Vulnerabilities",
            GridPosition { h: 4, w: 6, x: 6, y: y_pos },
            datasource.clone(),
            self.critical_count_query(),
        ));
        panel_id += 1;

        panels.push(self.create_stat_panel(
            panel_id,
            "High Vulnerabilities",
            GridPosition { h: 4, w: 6, x: 12, y: y_pos },
            datasource.clone(),
            self.high_count_query(),
        ));
        panel_id += 1;

        panels.push(self.create_stat_panel(
            panel_id,
            "Hosts Affected",
            GridPosition { h: 4, w: 6, x: 18, y: y_pos },
            datasource.clone(),
            self.hosts_affected_query(),
        ));
        panel_id += 1;
        y_pos += 4;

        // Row 2: Charts
        panels.push(self.create_piechart_panel(
            panel_id,
            "Severity Distribution",
            GridPosition { h: 8, w: 8, x: 0, y: y_pos },
            datasource.clone(),
            self.severity_distribution_query(),
        ));
        panel_id += 1;

        panels.push(self.create_timeseries_panel(
            panel_id,
            "Findings Over Time",
            GridPosition { h: 8, w: 16, x: 8, y: y_pos },
            datasource.clone(),
            self.findings_over_time_query(),
        ));
        panel_id += 1;
        y_pos += 8;

        // Row 3: Tables
        panels.push(self.create_table_panel(
            panel_id,
            "Top Vulnerabilities",
            GridPosition { h: 10, w: 12, x: 0, y: y_pos },
            datasource.clone(),
            self.top_vulnerabilities_query(),
        ));
        panel_id += 1;

        panels.push(self.create_table_panel(
            panel_id,
            "Most Affected Hosts",
            GridPosition { h: 10, w: 12, x: 12, y: y_pos },
            datasource.clone(),
            self.most_affected_hosts_query(),
        ));
        panel_id += 1;
        y_pos += 10;

        // Row 4: Security events
        panels.push(self.create_timeseries_panel(
            panel_id,
            "Security Events",
            GridPosition { h: 8, w: 24, x: 0, y: y_pos },
            datasource.clone(),
            self.security_events_query(),
        ));

        GrafanaDashboard {
            id: None,
            uid,
            title: self.config.title.clone(),
            tags: self.config.tags.clone(),
            timezone: "browser".to_string(),
            schema_version: 38,
            version: 1,
            refresh: self.config.refresh.clone(),
            time: GrafanaTimeRange {
                from: self.config.time_from.clone(),
                to: self.config.time_to.clone(),
            },
            panels,
            templating: GrafanaTemplating {
                list: vec![
                    TemplateVariable {
                        name: "severity".to_string(),
                        label: Some("Severity".to_string()),
                        var_type: "custom".to_string(),
                        query: None,
                        current: Some(serde_json::json!({"text": "All", "value": "$__all"})),
                        options: vec![
                            serde_json::json!({"text": "All", "value": "$__all"}),
                            serde_json::json!({"text": "Critical", "value": "critical"}),
                            serde_json::json!({"text": "High", "value": "high"}),
                            serde_json::json!({"text": "Medium", "value": "medium"}),
                            serde_json::json!({"text": "Low", "value": "low"}),
                        ],
                        multi: true,
                        include_all: true,
                    },
                ],
            },
            annotations: GrafanaAnnotations { list: vec![] },
        }
    }

    /// Create stat panel
    fn create_stat_panel(
        &self,
        id: u32,
        title: &str,
        grid_pos: GridPosition,
        datasource: PanelDatasource,
        query: serde_json::Value,
    ) -> GrafanaPanel {
        GrafanaPanel {
            id,
            panel_type: "stat".to_string(),
            title: title.to_string(),
            description: None,
            grid_pos,
            datasource,
            targets: vec![PanelTarget {
                ref_id: "A".to_string(),
                query,
            }],
            field_config: FieldConfig {
                defaults: FieldDefaults {
                    color: ColorConfig { mode: "thresholds".to_string() },
                    unit: None,
                    min: None,
                    max: None,
                    thresholds: Thresholds {
                        mode: "absolute".to_string(),
                        steps: vec![
                            ThresholdStep { color: "green".to_string(), value: None },
                            ThresholdStep { color: "yellow".to_string(), value: Some(10.0) },
                            ThresholdStep { color: "red".to_string(), value: Some(50.0) },
                        ],
                    },
                    mappings: vec![],
                },
                overrides: vec![],
            },
            options: serde_json::json!({
                "reduceOptions": {
                    "values": false,
                    "calcs": ["lastNotNull"],
                    "fields": ""
                },
                "orientation": "auto",
                "textMode": "auto",
                "colorMode": "value",
                "graphMode": "area",
                "justifyMode": "auto"
            }),
        }
    }

    /// Create pie chart panel
    fn create_piechart_panel(
        &self,
        id: u32,
        title: &str,
        grid_pos: GridPosition,
        datasource: PanelDatasource,
        query: serde_json::Value,
    ) -> GrafanaPanel {
        GrafanaPanel {
            id,
            panel_type: "piechart".to_string(),
            title: title.to_string(),
            description: None,
            grid_pos,
            datasource,
            targets: vec![PanelTarget {
                ref_id: "A".to_string(),
                query,
            }],
            field_config: FieldConfig {
                defaults: FieldDefaults {
                    color: ColorConfig { mode: "palette-classic".to_string() },
                    unit: None,
                    min: None,
                    max: None,
                    thresholds: Thresholds {
                        mode: "absolute".to_string(),
                        steps: vec![ThresholdStep { color: "green".to_string(), value: None }],
                    },
                    mappings: vec![],
                },
                overrides: vec![
                    serde_json::json!({
                        "matcher": {"id": "byName", "options": "critical"},
                        "properties": [{"id": "color", "value": {"fixedColor": "#9b2c2c", "mode": "fixed"}}]
                    }),
                    serde_json::json!({
                        "matcher": {"id": "byName", "options": "high"},
                        "properties": [{"id": "color", "value": {"fixedColor": "#c53030", "mode": "fixed"}}]
                    }),
                    serde_json::json!({
                        "matcher": {"id": "byName", "options": "medium"},
                        "properties": [{"id": "color", "value": {"fixedColor": "#dd6b20", "mode": "fixed"}}]
                    }),
                    serde_json::json!({
                        "matcher": {"id": "byName", "options": "low"},
                        "properties": [{"id": "color", "value": {"fixedColor": "#d69e2e", "mode": "fixed"}}]
                    }),
                ],
            },
            options: serde_json::json!({
                "reduceOptions": {"values": true, "calcs": ["lastNotNull"], "fields": ""},
                "pieType": "donut",
                "tooltip": {"mode": "single", "sort": "none"},
                "legend": {"displayMode": "list", "placement": "right", "showLegend": true}
            }),
        }
    }

    /// Create time series panel
    fn create_timeseries_panel(
        &self,
        id: u32,
        title: &str,
        grid_pos: GridPosition,
        datasource: PanelDatasource,
        query: serde_json::Value,
    ) -> GrafanaPanel {
        GrafanaPanel {
            id,
            panel_type: "timeseries".to_string(),
            title: title.to_string(),
            description: None,
            grid_pos,
            datasource,
            targets: vec![PanelTarget {
                ref_id: "A".to_string(),
                query,
            }],
            field_config: FieldConfig {
                defaults: FieldDefaults {
                    color: ColorConfig { mode: "palette-classic".to_string() },
                    unit: None,
                    min: Some(0.0),
                    max: None,
                    thresholds: Thresholds {
                        mode: "absolute".to_string(),
                        steps: vec![ThresholdStep { color: "green".to_string(), value: None }],
                    },
                    mappings: vec![],
                },
                overrides: vec![],
            },
            options: serde_json::json!({
                "tooltip": {"mode": "single", "sort": "none"},
                "legend": {"displayMode": "list", "placement": "bottom", "showLegend": true}
            }),
        }
    }

    /// Create table panel
    fn create_table_panel(
        &self,
        id: u32,
        title: &str,
        grid_pos: GridPosition,
        datasource: PanelDatasource,
        query: serde_json::Value,
    ) -> GrafanaPanel {
        GrafanaPanel {
            id,
            panel_type: "table".to_string(),
            title: title.to_string(),
            description: None,
            grid_pos,
            datasource,
            targets: vec![PanelTarget {
                ref_id: "A".to_string(),
                query,
            }],
            field_config: FieldConfig {
                defaults: FieldDefaults {
                    color: ColorConfig { mode: "thresholds".to_string() },
                    unit: None,
                    min: None,
                    max: None,
                    thresholds: Thresholds {
                        mode: "absolute".to_string(),
                        steps: vec![ThresholdStep { color: "green".to_string(), value: None }],
                    },
                    mappings: vec![],
                },
                overrides: vec![],
            },
            options: serde_json::json!({
                "showHeader": true,
                "sortBy": [{"displayName": "Count", "desc": true}]
            }),
        }
    }

    // Query builders for Elasticsearch
    fn total_findings_query(&self) -> serde_json::Value {
        serde_json::json!({
            "query": "event.kind:alert",
            "metrics": [{"type": "count", "id": "1"}],
            "bucketAggs": []
        })
    }

    fn critical_count_query(&self) -> serde_json::Value {
        serde_json::json!({
            "query": "event.kind:alert AND vulnerability.severity:critical",
            "metrics": [{"type": "count", "id": "1"}],
            "bucketAggs": []
        })
    }

    fn high_count_query(&self) -> serde_json::Value {
        serde_json::json!({
            "query": "event.kind:alert AND vulnerability.severity:high",
            "metrics": [{"type": "count", "id": "1"}],
            "bucketAggs": []
        })
    }

    fn hosts_affected_query(&self) -> serde_json::Value {
        serde_json::json!({
            "query": "event.kind:alert",
            "metrics": [{"type": "cardinality", "id": "1", "field": "host.hostname.keyword"}],
            "bucketAggs": []
        })
    }

    fn severity_distribution_query(&self) -> serde_json::Value {
        serde_json::json!({
            "query": "event.kind:alert",
            "metrics": [{"type": "count", "id": "1"}],
            "bucketAggs": [
                {"type": "terms", "id": "2", "field": "vulnerability.severity.keyword", "settings": {"size": "5"}}
            ]
        })
    }

    fn findings_over_time_query(&self) -> serde_json::Value {
        serde_json::json!({
            "query": "event.kind:alert",
            "metrics": [{"type": "count", "id": "1"}],
            "bucketAggs": [
                {"type": "date_histogram", "id": "2", "field": "@timestamp", "settings": {"interval": "1h"}}
            ]
        })
    }

    fn top_vulnerabilities_query(&self) -> serde_json::Value {
        serde_json::json!({
            "query": "event.kind:alert",
            "metrics": [{"type": "count", "id": "1"}],
            "bucketAggs": [
                {"type": "terms", "id": "2", "field": "vulnerability.id.keyword", "settings": {"size": "10"}}
            ]
        })
    }

    fn most_affected_hosts_query(&self) -> serde_json::Value {
        serde_json::json!({
            "query": "event.kind:alert",
            "metrics": [{"type": "count", "id": "1"}],
            "bucketAggs": [
                {"type": "terms", "id": "2", "field": "host.hostname.keyword", "settings": {"size": "10"}}
            ]
        })
    }

    fn security_events_query(&self) -> serde_json::Value {
        serde_json::json!({
            "query": "event.kind:event",
            "metrics": [{"type": "count", "id": "1"}],
            "bucketAggs": [
                {"type": "date_histogram", "id": "2", "field": "@timestamp", "settings": {"interval": "15m"}},
                {"type": "terms", "id": "3", "field": "event.category.keyword", "settings": {"size": "5"}}
            ]
        })
    }

    /// Export dashboard to JSON
    pub fn to_json(&self, dashboard: &GrafanaDashboard) -> ExportResult<String> {
        serde_json::to_string_pretty(dashboard)
            .map_err(|e| ExportError::SerializationError(e.to_string()))
    }

    /// Generate dashboard provisioning file
    pub fn provisioning_config(&self, dashboard_path: &str) -> serde_json::Value {
        serde_json::json!({
            "apiVersion": 1,
            "providers": [
                {
                    "name": "WIA Security Dashboards",
                    "orgId": 1,
                    "folder": "WIA Security",
                    "type": "file",
                    "disableDeletion": false,
                    "updateIntervalSeconds": 10,
                    "options": {
                        "path": dashboard_path
                    }
                }
            ]
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_dashboard() {
        let config = GrafanaConfig::default();
        let generator = GrafanaGenerator::new(config);

        let dashboard = generator.generate_security_dashboard();

        assert_eq!(dashboard.title, "WIA Security Dashboard");
        assert!(!dashboard.panels.is_empty());
        assert!(dashboard.panels.len() >= 8);
    }

    #[test]
    fn test_to_json() {
        let config = GrafanaConfig::default();
        let generator = GrafanaGenerator::new(config);

        let dashboard = generator.generate_security_dashboard();
        let json = generator.to_json(&dashboard).unwrap();

        assert!(json.contains("WIA Security Dashboard"));
        assert!(json.contains("panels"));
    }

    #[test]
    fn test_panel_types() {
        let config = GrafanaConfig::default();
        let generator = GrafanaGenerator::new(config);

        let dashboard = generator.generate_security_dashboard();

        let panel_types: Vec<_> = dashboard.panels.iter().map(|p| p.panel_type.as_str()).collect();
        assert!(panel_types.contains(&"stat"));
        assert!(panel_types.contains(&"piechart"));
        assert!(panel_types.contains(&"timeseries"));
        assert!(panel_types.contains(&"table"));
    }
}
