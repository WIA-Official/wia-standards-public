//! WIA Security Event Builder

use chrono::Utc;
use serde_json::json;
use uuid::Uuid;

use crate::types::*;

// ============================================================================
// Alert Builder
// ============================================================================

/// Builder for alert events
pub struct AlertBuilder {
    event: WiaSecurityEvent,
    alert_data: serde_json::Map<String, serde_json::Value>,
}

impl AlertBuilder {
    pub fn new() -> Self {
        let mut event = WiaSecurityEvent::new(
            EventType::Alert,
            Source::new(SourceType::Custom, "unknown"),
            json!({}),
        );
        event.severity = 5.0;

        Self {
            event,
            alert_data: serde_json::Map::new(),
        }
    }

    pub fn id(mut self, id: Uuid) -> Self {
        self.event.id = id;
        self
    }

    pub fn timestamp(mut self, ts: impl Into<String>) -> Self {
        self.event.timestamp = ts.into();
        self
    }

    pub fn severity(mut self, severity: f64) -> Self {
        self.event.severity = severity.clamp(0.0, 10.0);
        self
    }

    pub fn source(mut self, source: Source) -> Self {
        self.event.source = source;
        self
    }

    pub fn context(mut self, context: Context) -> Self {
        self.event.context = Some(context);
        self
    }

    pub fn mitre(mut self, mitre: Mitre) -> Self {
        self.event.mitre = Some(mitre);
        self
    }

    pub fn tactic(mut self, id: impl Into<String>, name: Option<&str>) -> Self {
        let mut mitre = self.event.mitre.take().unwrap_or_default();
        mitre.tactic = Some(id.into());
        if let Some(n) = name {
            mitre.tactic_name = Some(n.to_string());
        }
        self.event.mitre = Some(mitre);
        self
    }

    pub fn technique(mut self, id: impl Into<String>, name: Option<&str>) -> Self {
        let mut mitre = self.event.mitre.take().unwrap_or_default();
        mitre.technique = Some(id.into());
        if let Some(n) = name {
            mitre.technique_name = Some(n.to_string());
        }
        self.event.mitre = Some(mitre);
        self
    }

    pub fn meta(mut self, meta: Meta) -> Self {
        self.event.meta = Some(meta);
        self
    }

    pub fn confidence(mut self, confidence: f64) -> Self {
        let mut meta = self.event.meta.take().unwrap_or_default();
        meta.confidence = Some(confidence.clamp(0.0, 1.0));
        self.event.meta = Some(meta);
        self
    }

    pub fn tags(mut self, tags: Vec<String>) -> Self {
        let mut meta = self.event.meta.take().unwrap_or_default();
        meta.tags = Some(tags);
        self.event.meta = Some(meta);
        self
    }

    pub fn alert_id(mut self, alert_id: impl Into<String>) -> Self {
        self.alert_data
            .insert("alert_id".to_string(), json!(alert_id.into()));
        self
    }

    pub fn title(mut self, title: impl Into<String>) -> Self {
        self.alert_data
            .insert("title".to_string(), json!(title.into()));
        self
    }

    pub fn description(mut self, description: impl Into<String>) -> Self {
        self.alert_data
            .insert("description".to_string(), json!(description.into()));
        self
    }

    pub fn category(mut self, category: impl Into<String>) -> Self {
        self.alert_data
            .insert("category".to_string(), json!(category.into()));
        self
    }

    pub fn status(mut self, status: impl Into<String>) -> Self {
        self.alert_data
            .insert("status".to_string(), json!(status.into()));
        self
    }

    pub fn priority(mut self, priority: impl Into<String>) -> Self {
        self.alert_data
            .insert("priority".to_string(), json!(priority.into()));
        self
    }

    pub fn detection_rule(
        mut self,
        id: impl Into<String>,
        name: impl Into<String>,
        version: Option<&str>,
    ) -> Self {
        let mut rule = json!({
            "id": id.into(),
            "name": name.into()
        });
        if let Some(v) = version {
            rule["version"] = json!(v);
        }
        self.alert_data
            .insert("detection_rule".to_string(), rule);
        self
    }

    pub fn add_indicator(
        mut self,
        indicator_type: impl Into<String>,
        value: impl Into<String>,
        context: Option<&str>,
    ) -> Self {
        let indicators = self
            .alert_data
            .entry("indicators".to_string())
            .or_insert_with(|| json!([]));

        if let Some(arr) = indicators.as_array_mut() {
            let mut ind = json!({
                "type": indicator_type.into(),
                "value": value.into()
            });
            if let Some(ctx) = context {
                ind["context"] = json!(ctx);
            }
            arr.push(ind);
        }
        self
    }

    pub fn build(mut self) -> Result<WiaSecurityEvent, &'static str> {
        // Validate required fields
        if !self.alert_data.contains_key("alert_id") {
            return Err("alert_id is required");
        }
        if !self.alert_data.contains_key("title") {
            return Err("title is required");
        }
        if !self.alert_data.contains_key("category") {
            return Err("category is required");
        }
        if !self.alert_data.contains_key("status") {
            return Err("status is required");
        }
        if !self.alert_data.contains_key("priority") {
            return Err("priority is required");
        }

        self.event.data = serde_json::Value::Object(self.alert_data);
        Ok(self.event)
    }
}

impl Default for AlertBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Threat Intel Builder
// ============================================================================

/// Builder for threat intelligence events
pub struct ThreatIntelBuilder {
    event: WiaSecurityEvent,
    threat_data: serde_json::Map<String, serde_json::Value>,
}

impl ThreatIntelBuilder {
    pub fn new() -> Self {
        let event = WiaSecurityEvent::new(
            EventType::ThreatIntel,
            Source::new(SourceType::Custom, "unknown"),
            json!({}),
        );

        Self {
            event,
            threat_data: serde_json::Map::new(),
        }
    }

    pub fn source(mut self, source: Source) -> Self {
        self.event.source = source;
        self
    }

    pub fn severity(mut self, severity: f64) -> Self {
        self.event.severity = severity.clamp(0.0, 10.0);
        self
    }

    pub fn threat_type(mut self, threat_type: impl Into<String>) -> Self {
        self.threat_data
            .insert("threat_type".to_string(), json!(threat_type.into()));
        self
    }

    pub fn threat_name(mut self, name: impl Into<String>) -> Self {
        self.threat_data
            .insert("threat_name".to_string(), json!(name.into()));
        self
    }

    pub fn threat_family(mut self, family: impl Into<String>) -> Self {
        self.threat_data
            .insert("threat_family".to_string(), json!(family.into()));
        self
    }

    pub fn status(mut self, status: impl Into<String>) -> Self {
        self.threat_data
            .insert("status".to_string(), json!(status.into()));
        self
    }

    pub fn target_sectors(mut self, sectors: Vec<String>) -> Self {
        self.threat_data
            .insert("target_sectors".to_string(), json!(sectors));
        self
    }

    pub fn target_countries(mut self, countries: Vec<String>) -> Self {
        self.threat_data
            .insert("target_countries".to_string(), json!(countries));
        self
    }

    pub fn build(mut self) -> Result<WiaSecurityEvent, &'static str> {
        if !self.threat_data.contains_key("threat_type") {
            return Err("threat_type is required");
        }
        if !self.threat_data.contains_key("threat_name") {
            return Err("threat_name is required");
        }
        if !self.threat_data.contains_key("status") {
            return Err("status is required");
        }

        self.event.data = serde_json::Value::Object(self.threat_data);
        Ok(self.event)
    }
}

impl Default for ThreatIntelBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Vulnerability Builder
// ============================================================================

/// Builder for vulnerability events
pub struct VulnerabilityBuilder {
    event: WiaSecurityEvent,
    vuln_data: serde_json::Map<String, serde_json::Value>,
}

impl VulnerabilityBuilder {
    pub fn new() -> Self {
        let event = WiaSecurityEvent::new(
            EventType::Vulnerability,
            Source::new(SourceType::Scanner, "unknown"),
            json!({}),
        );

        Self {
            event,
            vuln_data: serde_json::Map::new(),
        }
    }

    pub fn source(mut self, source: Source) -> Self {
        self.event.source = source;
        self
    }

    pub fn severity(mut self, severity: f64) -> Self {
        self.event.severity = severity.clamp(0.0, 10.0);
        self
    }

    pub fn vuln_id(mut self, vuln_id: impl Into<String>) -> Self {
        self.vuln_data
            .insert("vuln_id".to_string(), json!(vuln_id.into()));
        self
    }

    pub fn title(mut self, title: impl Into<String>) -> Self {
        self.vuln_data
            .insert("title".to_string(), json!(title.into()));
        self
    }

    pub fn cvss(
        mut self,
        version: impl Into<String>,
        score: f64,
        severity: impl Into<String>,
    ) -> Self {
        self.vuln_data.insert(
            "cvss".to_string(),
            json!({
                "version": version.into(),
                "score": score,
                "severity": severity.into()
            }),
        );
        self
    }

    pub fn cwe(mut self, cwe_ids: Vec<String>) -> Self {
        self.vuln_data.insert("cwe".to_string(), json!(cwe_ids));
        self
    }

    pub fn exploit_available(mut self, available: bool) -> Self {
        self.vuln_data
            .insert("exploit_available".to_string(), json!(available));
        self
    }

    pub fn patch_available(mut self, available: bool) -> Self {
        self.vuln_data
            .insert("patch_available".to_string(), json!(available));
        self
    }

    pub fn build(mut self) -> Result<WiaSecurityEvent, &'static str> {
        if !self.vuln_data.contains_key("vuln_id") {
            return Err("vuln_id is required");
        }
        if !self.vuln_data.contains_key("title") {
            return Err("title is required");
        }
        if !self.vuln_data.contains_key("cvss") {
            return Err("cvss is required");
        }

        self.event.data = serde_json::Value::Object(self.vuln_data);
        Ok(self.event)
    }
}

impl Default for VulnerabilityBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Factory Functions
// ============================================================================

/// Create an alert builder
pub fn create_alert() -> AlertBuilder {
    AlertBuilder::new()
}

/// Create a threat intel builder
pub fn create_threat_intel() -> ThreatIntelBuilder {
    ThreatIntelBuilder::new()
}

/// Create a vulnerability builder
pub fn create_vulnerability() -> VulnerabilityBuilder {
    VulnerabilityBuilder::new()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_alert_builder() {
        let event = AlertBuilder::new()
            .source(Source::new(SourceType::Siem, "Splunk"))
            .alert_id("ALERT-001")
            .title("Test Alert")
            .category("malware")
            .status("new")
            .priority("high")
            .severity(7.0)
            .build()
            .unwrap();

        assert_eq!(event.event_type, EventType::Alert);
        assert_eq!(event.severity, 7.0);
    }
}
