//! Dashboard Integration Adapter
//!
//! Real-time dashboard and visualization adapters

use async_trait::async_trait;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::time::Duration;

use super::adapter::*;
use crate::error::{HealthError, Result};
use crate::types::HealthProfile;

/// Widget types for dashboard visualization
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WidgetType {
    /// Line chart for time series data
    LineChart,
    /// Gauge chart for single values
    GaugeChart,
    /// Number card for key metrics
    NumberCard,
    /// Data table
    Table,
    /// Heatmap visualization
    Heatmap,
    /// 3D digital twin
    DigitalTwin3D,
    /// Alert list
    AlertList,
    /// Progress bar
    ProgressBar,
    /// Sparkline mini chart
    Sparkline,
}

/// Dashboard widget configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Widget {
    /// Unique widget ID
    pub id: String,
    /// Widget type
    pub widget_type: WidgetType,
    /// Data source path (e.g., "biomarkers.cardiovascular.heart_rate")
    pub data_source: String,
    /// Refresh rate
    #[serde(with = "humantime_serde")]
    pub refresh_rate: Duration,
    /// Widget title
    pub title: Option<String>,
    /// Additional configuration
    pub config: serde_json::Value,
}

/// Dashboard update message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DashboardUpdate {
    /// Widget ID
    pub widget_id: String,
    /// Update data
    pub data: serde_json::Value,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
    /// Update type
    pub update_type: UpdateType,
}

/// Update types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UpdateType {
    /// Full data refresh
    Full,
    /// Incremental update
    Delta,
    /// Append to existing data
    Append,
}

/// Chart data series
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChartSeries {
    /// Series name
    pub name: String,
    /// Data points
    pub data: Vec<DataPoint>,
}

/// Chart data point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataPoint {
    /// X value (usually timestamp)
    pub x: serde_json::Value,
    /// Y value
    pub y: f64,
}

/// Threshold configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Thresholds {
    /// Low warning threshold
    pub low: Option<f64>,
    /// High warning threshold
    pub high: Option<f64>,
    /// Critical low threshold
    pub critical_low: Option<f64>,
    /// Critical high threshold
    pub critical_high: Option<f64>,
}

/// Dashboard adapter
pub struct DashboardAdapter {
    name: String,
    config: AdapterConfig,
    initialized: bool,
    widgets: Vec<Widget>,
}

impl DashboardAdapter {
    /// Create new dashboard adapter
    pub fn new() -> Self {
        Self {
            name: "Dashboard Adapter".to_string(),
            config: AdapterConfig::default(),
            initialized: false,
            widgets: Vec::new(),
        }
    }

    /// Create with WebSocket URL
    pub fn with_url(url: &str) -> Self {
        let mut adapter = Self::new();
        adapter.config.base_url = Some(url.to_string());
        adapter
    }

    /// Register a widget
    pub fn register_widget(&mut self, widget: Widget) {
        self.widgets.push(widget);
    }

    /// Remove a widget
    pub fn unregister_widget(&mut self, widget_id: &str) {
        self.widgets.retain(|w| w.id != widget_id);
    }

    /// Get registered widgets
    pub fn widgets(&self) -> &[Widget] {
        &self.widgets
    }

    /// Create update from profile for a widget
    pub fn create_update(
        &self,
        widget: &Widget,
        profile: &HealthProfile,
    ) -> Option<DashboardUpdate> {
        let value = self.extract_value(&widget.data_source, profile)?;

        Some(DashboardUpdate {
            widget_id: widget.id.clone(),
            data: value,
            timestamp: Utc::now(),
            update_type: UpdateType::Delta,
        })
    }

    /// Extract value from profile by path
    fn extract_value(&self, path: &str, profile: &HealthProfile) -> Option<serde_json::Value> {
        let parts: Vec<&str> = path.split('.').collect();

        match parts.as_slice() {
            // Note: cardiovascular_markers not in current schema
            // Placeholder for future cardiovascular metrics
            ["biomarkers", "cardiovascular", "heart_rate"] => None,
            ["biomarkers", "metabolic", "glucose"] => {
                profile
                    .biomarkers
                    .as_ref()
                    .and_then(|b| b.metabolic_markers.as_ref())
                    .and_then(|m| m.glucose.as_ref())
                    .map(|g| {
                        serde_json::json!({
                            "value": g.value,
                            "unit": g.unit,
                            "timestamp": g.timestamp
                        })
                    })
            }
            ["aging_clocks", "biological_age"] => {
                profile
                    .biomarkers
                    .as_ref()
                    .and_then(|b| b.aging_clocks.as_ref())
                    .and_then(|a| a.biological_age)
                    .map(|age| serde_json::json!({ "value": age }))
            }
            _ => None,
        }
    }

    /// Generate chart data from profile history
    pub fn generate_chart_data(
        &self,
        data_source: &str,
        _profile: &HealthProfile,
    ) -> ChartSeries {
        // In a real implementation, this would query historical data
        // For now, return mock data
        ChartSeries {
            name: data_source.to_string(),
            data: vec![
                DataPoint {
                    x: serde_json::json!("2025-12-14T10:00:00Z"),
                    y: 72.0,
                },
                DataPoint {
                    x: serde_json::json!("2025-12-14T10:05:00Z"),
                    y: 75.0,
                },
                DataPoint {
                    x: serde_json::json!("2025-12-14T10:10:00Z"),
                    y: 71.0,
                },
            ],
        }
    }
}

impl Default for DashboardAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl IntegrationAdapter for DashboardAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::Dashboard
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn is_available(&self) -> bool {
        self.initialized
    }

    async fn initialize(&mut self, config: AdapterConfig) -> Result<()> {
        self.config = config;
        self.initialized = true;
        Ok(())
    }

    async fn shutdown(&mut self) -> Result<()> {
        self.initialized = false;
        self.widgets.clear();
        Ok(())
    }

    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities {
            can_import: false,
            can_export: true,
            can_stream: true,
            supports_auth: true,
            data_types: vec![
                "chart".to_string(),
                "gauge".to_string(),
                "table".to_string(),
            ],
        }
    }
}

#[async_trait]
impl ExportAdapter for DashboardAdapter {
    async fn export(&self, profile: &HealthProfile, _options: ExportOptions) -> Result<ExportResult> {
        if !self.initialized {
            return Err(HealthError::adapter("Dashboard adapter not initialized"));
        }

        let mut updates_sent = 0;

        for widget in &self.widgets {
            if self.create_update(widget, profile).is_some() {
                updates_sent += 1;
                // In a real implementation, send via WebSocket
            }
        }

        Ok(ExportResult {
            success: true,
            records_exported: updates_sent,
            destination: self
                .config
                .base_url
                .clone()
                .unwrap_or_else(|| "local".to_string()),
            resource_ids: self.widgets.iter().map(|w| w.id.clone()).collect(),
            exported_at: Utc::now(),
        })
    }

    async fn export_data(&self, data: ExportData, _options: ExportOptions) -> Result<ExportResult> {
        if !self.initialized {
            return Err(HealthError::adapter("Dashboard adapter not initialized"));
        }

        Ok(ExportResult {
            success: true,
            records_exported: 1,
            destination: self
                .config
                .base_url
                .clone()
                .unwrap_or_else(|| "local".to_string()),
            resource_ids: vec![data.data_type],
            exported_at: Utc::now(),
        })
    }
}

/// Serialize/deserialize Duration with humantime
mod humantime_serde {
    use serde::{self, Deserialize, Deserializer, Serializer};
    use std::time::Duration;

    pub fn serialize<S>(duration: &Duration, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_u64(duration.as_millis() as u64)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Duration, D::Error>
    where
        D: Deserializer<'de>,
    {
        let ms = u64::deserialize(deserializer)?;
        Ok(Duration::from_millis(ms))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_widget_creation() {
        let widget = Widget {
            id: "hr-chart".to_string(),
            widget_type: WidgetType::LineChart,
            data_source: "biomarkers.cardiovascular.heart_rate".to_string(),
            refresh_rate: Duration::from_secs(1),
            title: Some("Heart Rate".to_string()),
            config: serde_json::json!({}),
        };

        assert_eq!(widget.id, "hr-chart");
    }

    #[tokio::test]
    async fn test_dashboard_adapter() {
        let mut adapter = DashboardAdapter::new();
        adapter.initialize(AdapterConfig::default()).await.unwrap();

        adapter.register_widget(Widget {
            id: "test-widget".to_string(),
            widget_type: WidgetType::GaugeChart,
            data_source: "test".to_string(),
            refresh_rate: Duration::from_secs(5),
            title: None,
            config: serde_json::json!({}),
        });

        assert_eq!(adapter.widgets().len(), 1);
        assert!(adapter.is_available().await);
    }
}
