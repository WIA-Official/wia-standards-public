//! Dashboard & Metrics Integration
//!
//! Security metrics aggregation and compliance tracking

use crate::types::{EventType, WiaSecurityEvent};
use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// Dashboard Aggregator
// ============================================================================

/// Dashboard aggregator configuration
#[derive(Debug, Clone)]
pub struct DashboardConfig {
    pub time_window_hours: u32,
}

impl Default for DashboardConfig {
    fn default() -> Self {
        Self {
            time_window_hours: 24,
        }
    }
}

/// Dashboard aggregator for security metrics
pub struct DashboardAggregator {
    config: DashboardConfig,
    events: Vec<WiaSecurityEvent>,
}

impl DashboardAggregator {
    /// Create a new dashboard aggregator
    pub fn new(config: DashboardConfig) -> Self {
        Self {
            config,
            events: Vec::new(),
        }
    }

    /// Add events to the aggregator
    pub fn add_events(&mut self, events: Vec<WiaSecurityEvent>) {
        self.events.extend(events);
    }

    /// Add a single event
    pub fn add_event(&mut self, event: WiaSecurityEvent) {
        self.events.push(event);
    }

    /// Clear all events
    pub fn clear(&mut self) {
        self.events.clear();
    }

    /// Get aggregated metrics
    pub fn get_metrics(&self) -> DashboardMetrics {
        let now = Utc::now();
        let window_start = now - Duration::hours(self.config.time_window_hours as i64);

        let filtered_events: Vec<_> = self
            .events
            .iter()
            .filter(|e| {
                DateTime::parse_from_rfc3339(&e.timestamp)
                    .map(|dt| dt.with_timezone(&Utc) >= window_start)
                    .unwrap_or(false)
            })
            .collect();

        let total_events = filtered_events.len();

        // Severity distribution
        let mut severity_distribution = SeverityDistribution::default();
        for event in &filtered_events {
            if event.severity >= 9.0 {
                severity_distribution.critical += 1;
            } else if event.severity >= 7.0 {
                severity_distribution.high += 1;
            } else if event.severity >= 4.0 {
                severity_distribution.medium += 1;
            } else if event.severity >= 1.0 {
                severity_distribution.low += 1;
            } else {
                severity_distribution.info += 1;
            }
        }

        // Event type distribution
        let mut event_type_distribution = HashMap::new();
        for event in &filtered_events {
            *event_type_distribution
                .entry(event.event_type.to_string())
                .or_insert(0) += 1;
        }

        // Top MITRE techniques
        let mut mitre_techniques: HashMap<String, usize> = HashMap::new();
        for event in &filtered_events {
            if let Some(mitre) = &event.mitre {
                if let Some(technique) = &mitre.technique {
                    *mitre_techniques.entry(technique.clone()).or_insert(0) += 1;
                }
            }
        }
        let mut top_mitre: Vec<_> = mitre_techniques.into_iter().collect();
        top_mitre.sort_by(|a, b| b.1.cmp(&a.1));
        let top_mitre_techniques: Vec<_> = top_mitre
            .into_iter()
            .take(10)
            .map(|(tech, count)| MitreTechniqueCount {
                technique: tech,
                count,
            })
            .collect();

        // Calculate MTTD (Mean Time to Detect)
        let mttd = self.calculate_mttd(&filtered_events);

        // Calculate risk score
        let risk_score = self.calculate_risk_score(&filtered_events);

        // Time series (hourly counts)
        let time_series = self.generate_time_series(&filtered_events);

        DashboardMetrics {
            total_events,
            time_window_hours: self.config.time_window_hours,
            severity_distribution,
            event_type_distribution,
            top_mitre_techniques,
            mean_time_to_detect_minutes: mttd,
            risk_score,
            time_series,
        }
    }

    fn calculate_mttd(&self, events: &[&WiaSecurityEvent]) -> Option<f64> {
        // In a real implementation, this would compare detection time vs. event time
        // For now, return a placeholder
        if events.is_empty() {
            None
        } else {
            Some(15.0) // 15 minutes average
        }
    }

    fn calculate_risk_score(&self, events: &[&WiaSecurityEvent]) -> f64 {
        if events.is_empty() {
            return 0.0;
        }

        let total_severity: f64 = events.iter().map(|e| e.severity).sum();
        let avg_severity = total_severity / events.len() as f64;

        // Weight by critical events
        let critical_count = events.iter().filter(|e| e.severity >= 9.0).count();
        let critical_weight = 1.0 + (critical_count as f64 * 0.1);

        (avg_severity * critical_weight).min(10.0)
    }

    fn generate_time_series(&self, events: &[&WiaSecurityEvent]) -> Vec<TimeSeriesPoint> {
        let mut hourly_counts: HashMap<i64, usize> = HashMap::new();

        for event in events {
            if let Ok(dt) = DateTime::parse_from_rfc3339(&event.timestamp) {
                let hour = dt.timestamp() / 3600 * 3600;
                *hourly_counts.entry(hour).or_insert(0) += 1;
            }
        }

        let mut points: Vec<_> = hourly_counts
            .into_iter()
            .map(|(timestamp, count)| TimeSeriesPoint {
                timestamp: DateTime::from_timestamp(timestamp, 0)
                    .unwrap_or(Utc::now())
                    .to_rfc3339(),
                count,
            })
            .collect();

        points.sort_by(|a, b| a.timestamp.cmp(&b.timestamp));
        points
    }
}

/// Dashboard metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DashboardMetrics {
    pub total_events: usize,
    pub time_window_hours: u32,
    pub severity_distribution: SeverityDistribution,
    pub event_type_distribution: HashMap<String, usize>,
    pub top_mitre_techniques: Vec<MitreTechniqueCount>,
    pub mean_time_to_detect_minutes: Option<f64>,
    pub risk_score: f64,
    pub time_series: Vec<TimeSeriesPoint>,
}

/// Severity distribution
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SeverityDistribution {
    pub critical: usize,
    pub high: usize,
    pub medium: usize,
    pub low: usize,
    pub info: usize,
}

/// MITRE technique count
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MitreTechniqueCount {
    pub technique: String,
    pub count: usize,
}

/// Time series data point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesPoint {
    pub timestamp: String,
    pub count: usize,
}

// ============================================================================
// Compliance Tracker
// ============================================================================

/// Compliance framework
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum ComplianceFramework {
    Nist,
    Cis,
    PciDss,
    Hipaa,
    Gdpr,
    Sox,
    Iso27001,
}

/// Compliance control
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComplianceControl {
    pub id: String,
    pub name: String,
    pub framework: ComplianceFramework,
    pub description: String,
}

/// Compliance tracker
pub struct ComplianceTracker {
    controls: HashMap<ComplianceFramework, Vec<ComplianceControl>>,
}

impl ComplianceTracker {
    /// Create a new compliance tracker
    pub fn new() -> Self {
        let mut controls = HashMap::new();

        // NIST controls mapping
        controls.insert(
            ComplianceFramework::Nist,
            vec![
                ComplianceControl {
                    id: "ID.AM".to_string(),
                    name: "Asset Management".to_string(),
                    framework: ComplianceFramework::Nist,
                    description: "Inventory and management of assets".to_string(),
                },
                ComplianceControl {
                    id: "PR.AC".to_string(),
                    name: "Access Control".to_string(),
                    framework: ComplianceFramework::Nist,
                    description: "Access to assets is managed".to_string(),
                },
                ComplianceControl {
                    id: "DE.AE".to_string(),
                    name: "Anomalies and Events".to_string(),
                    framework: ComplianceFramework::Nist,
                    description: "Anomalous activity is detected".to_string(),
                },
                ComplianceControl {
                    id: "DE.CM".to_string(),
                    name: "Security Continuous Monitoring".to_string(),
                    framework: ComplianceFramework::Nist,
                    description: "Information system and assets are monitored".to_string(),
                },
                ComplianceControl {
                    id: "RS.AN".to_string(),
                    name: "Analysis".to_string(),
                    framework: ComplianceFramework::Nist,
                    description: "Analysis is conducted to ensure response".to_string(),
                },
            ],
        );

        // CIS controls
        controls.insert(
            ComplianceFramework::Cis,
            vec![
                ComplianceControl {
                    id: "CIS-1".to_string(),
                    name: "Inventory of Hardware Assets".to_string(),
                    framework: ComplianceFramework::Cis,
                    description: "Actively manage hardware devices".to_string(),
                },
                ComplianceControl {
                    id: "CIS-6".to_string(),
                    name: "Maintenance and Analysis of Audit Logs".to_string(),
                    framework: ComplianceFramework::Cis,
                    description: "Collect and analyze audit logs".to_string(),
                },
                ComplianceControl {
                    id: "CIS-8".to_string(),
                    name: "Malware Defenses".to_string(),
                    framework: ComplianceFramework::Cis,
                    description: "Control installation and execution of malicious code".to_string(),
                },
            ],
        );

        Self { controls }
    }

    /// Map event to compliance controls
    pub fn map_event_to_controls(
        &self,
        event: &WiaSecurityEvent,
        framework: ComplianceFramework,
    ) -> Vec<ComplianceControl> {
        let framework_controls = match self.controls.get(&framework) {
            Some(c) => c,
            None => return vec![],
        };

        let mut matched_controls = vec![];

        match event.event_type {
            EventType::Alert | EventType::ThreatIntel => {
                // Maps to detection controls
                matched_controls.extend(
                    framework_controls
                        .iter()
                        .filter(|c| c.id.contains("DE") || c.id.contains("CIS-8"))
                        .cloned(),
                );
            }
            EventType::AuthEvent => {
                // Maps to access control
                matched_controls.extend(
                    framework_controls
                        .iter()
                        .filter(|c| c.id.contains("AC") || c.id.contains("PR"))
                        .cloned(),
                );
            }
            EventType::Vulnerability => {
                // Maps to asset management and continuous monitoring
                matched_controls.extend(
                    framework_controls
                        .iter()
                        .filter(|c| c.id.contains("AM") || c.id.contains("CM"))
                        .cloned(),
                );
            }
            EventType::Incident => {
                // Maps to response controls
                matched_controls.extend(
                    framework_controls
                        .iter()
                        .filter(|c| c.id.contains("RS") || c.id.contains("AN"))
                        .cloned(),
                );
            }
            _ => {
                // Network and endpoint events map to monitoring
                matched_controls.extend(
                    framework_controls
                        .iter()
                        .filter(|c| c.id.contains("CM") || c.id.contains("CIS-6"))
                        .cloned(),
                );
            }
        }

        matched_controls
    }

    /// Calculate compliance score from events
    pub fn calculate_score_from_events(
        &self,
        events: &[WiaSecurityEvent],
        framework: ComplianceFramework,
    ) -> ComplianceScore {
        let framework_controls = match self.controls.get(&framework) {
            Some(c) => c,
            None => return ComplianceScore::default(),
        };

        let total_controls = framework_controls.len();
        let mut covered_controls: std::collections::HashSet<String> =
            std::collections::HashSet::new();

        for event in events {
            let mapped = self.map_event_to_controls(event, framework);
            for control in mapped {
                covered_controls.insert(control.id);
            }
        }

        let covered_count = covered_controls.len();
        let score = if total_controls > 0 {
            (covered_count as f64 / total_controls as f64) * 100.0
        } else {
            0.0
        };

        ComplianceScore {
            framework: Some(framework),
            total_controls,
            covered_controls: covered_count,
            score,
            covered_control_ids: covered_controls.into_iter().collect(),
        }
    }

    /// Get all controls for a framework
    pub fn get_framework_controls(&self, framework: ComplianceFramework) -> Vec<ComplianceControl> {
        self.controls.get(&framework).cloned().unwrap_or_default()
    }
}

impl Default for ComplianceTracker {
    fn default() -> Self {
        Self::new()
    }
}

/// Compliance score
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ComplianceScore {
    pub framework: Option<ComplianceFramework>,
    pub total_controls: usize,
    pub covered_controls: usize,
    pub score: f64,
    pub covered_control_ids: Vec<String>,
}

// ============================================================================
// Grafana Dashboard Generator
// ============================================================================

/// Generate Grafana dashboard JSON
pub fn generate_grafana_dashboard() -> serde_json::Value {
    serde_json::json!({
        "dashboard": {
            "title": "WIA Security Dashboard",
            "tags": ["security", "wia"],
            "timezone": "browser",
            "panels": [
                {
                    "id": 1,
                    "title": "Severity Distribution",
                    "type": "piechart",
                    "targets": [{
                        "expr": "count by (severity) (wia_security_events)",
                        "legendFormat": "{{severity}}"
                    }],
                    "gridPos": { "h": 8, "w": 12, "x": 0, "y": 0 }
                },
                {
                    "id": 2,
                    "title": "Events Over Time",
                    "type": "graph",
                    "targets": [{
                        "expr": "sum(rate(wia_security_events_total[5m]))",
                        "legendFormat": "Events/sec"
                    }],
                    "gridPos": { "h": 8, "w": 12, "x": 12, "y": 0 }
                },
                {
                    "id": 3,
                    "title": "Critical Findings",
                    "type": "table",
                    "targets": [{
                        "expr": "wia_security_events{severity=\"critical\"}"
                    }],
                    "gridPos": { "h": 10, "w": 24, "x": 0, "y": 8 }
                },
                {
                    "id": 4,
                    "title": "Risk Score",
                    "type": "gauge",
                    "targets": [{
                        "expr": "wia_security_risk_score"
                    }],
                    "gridPos": { "h": 6, "w": 8, "x": 0, "y": 18 }
                },
                {
                    "id": 5,
                    "title": "Top MITRE Techniques",
                    "type": "barchart",
                    "targets": [{
                        "expr": "topk(10, sum by (technique) (wia_security_mitre_techniques))"
                    }],
                    "gridPos": { "h": 6, "w": 16, "x": 8, "y": 18 }
                }
            ],
            "refresh": "30s",
            "time": {
                "from": "now-24h",
                "to": "now"
            }
        },
        "overwrite": true
    })
}

// ============================================================================
// Factory Functions
// ============================================================================

/// Create dashboard aggregator
pub fn create_dashboard_aggregator(config: DashboardConfig) -> DashboardAggregator {
    DashboardAggregator::new(config)
}

/// Create compliance tracker
pub fn create_compliance_tracker() -> ComplianceTracker {
    ComplianceTracker::new()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Source, SourceType};

    #[test]
    fn test_dashboard_metrics() {
        let config = DashboardConfig {
            time_window_hours: 24,
        };
        let mut aggregator = DashboardAggregator::new(config);

        let event = WiaSecurityEvent::new(
            EventType::Alert,
            Source::new(SourceType::Siem, "Test"),
            serde_json::json!({ "title": "Test Alert" }),
        );

        aggregator.add_event(event);
        let metrics = aggregator.get_metrics();

        assert_eq!(metrics.total_events, 1);
    }

    #[test]
    fn test_compliance_tracker() {
        let tracker = ComplianceTracker::new();
        let controls = tracker.get_framework_controls(ComplianceFramework::Nist);
        assert!(!controls.is_empty());
    }
}
