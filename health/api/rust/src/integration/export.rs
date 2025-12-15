//! Export Adapter
//!
//! Data export in various formats (CSV, JSON, PDF)

use async_trait::async_trait;
use chrono::Utc;

use super::adapter::*;
use crate::error::{HealthError, Result};
use crate::types::HealthProfile;

/// Export adapter for various formats
pub struct ExportFormatAdapter {
    name: String,
    config: AdapterConfig,
    initialized: bool,
}

impl ExportFormatAdapter {
    /// Create new export adapter
    pub fn new() -> Self {
        Self {
            name: "Export Format Adapter".to_string(),
            config: AdapterConfig::default(),
            initialized: false,
        }
    }

    /// Export profile to JSON
    pub fn to_json(&self, profile: &HealthProfile, pretty: bool) -> Result<String> {
        if pretty {
            serde_json::to_string_pretty(profile)
                .map_err(|e| HealthError::SerializationError(e.to_string()))
        } else {
            serde_json::to_string(profile)
                .map_err(|e| HealthError::SerializationError(e.to_string()))
        }
    }

    /// Export profile to CSV (flattened format)
    pub fn to_csv(&self, profile: &HealthProfile) -> Result<String> {
        let mut csv = String::new();

        // Header
        csv.push_str("category,field,value,unit,timestamp\n");

        // Subject info
        csv.push_str(&format!(
            "subject,id,{},,,\n",
            profile.subject.id
        ));
        if let Some(ref year) = profile.subject.birth_year {
            csv.push_str(&format!("subject,birth_year,{},,,\n", year));
        }

        // Biomarkers
        if let Some(ref biomarkers) = profile.biomarkers {
            // Aging clocks
            if let Some(ref clocks) = biomarkers.aging_clocks {
                if let Some(age) = clocks.biological_age {
                    csv.push_str(&format!(
                        "aging_clocks,biological_age,{},years,{}\n",
                        age,
                        clocks.calculated_at.map(|t| t.to_rfc3339()).unwrap_or_default()
                    ));
                }
                if let Some(delta) = clocks.age_delta {
                    csv.push_str(&format!("aging_clocks,age_delta,{},years,\n", delta));
                }
            }

            // Inflammatory markers
            if let Some(ref inflammatory) = biomarkers.inflammatory_markers {
                if let Some(ref crp) = inflammatory.crp {
                    csv.push_str(&format!(
                        "inflammatory,crp,{},{},{}\n",
                        crp.value,
                        crp.unit,
                        crp.timestamp.map(|t| t.to_rfc3339()).unwrap_or_default()
                    ));
                }
            }

            // Metabolic markers
            if let Some(ref metabolic) = biomarkers.metabolic_markers {
                if let Some(ref glucose) = metabolic.glucose {
                    csv.push_str(&format!(
                        "metabolic,glucose,{},{},{}\n",
                        glucose.value,
                        glucose.unit,
                        glucose.timestamp.map(|t| t.to_rfc3339()).unwrap_or_default()
                    ));
                }
                if let Some(ref hba1c) = metabolic.hba1c {
                    csv.push_str(&format!(
                        "metabolic,hba1c,{},{},{}\n",
                        hba1c.value,
                        hba1c.unit,
                        hba1c.timestamp.map(|t| t.to_rfc3339()).unwrap_or_default()
                    ));
                }
            }
        }

        Ok(csv)
    }

    /// Generate PDF report (returns HTML that can be converted to PDF)
    pub fn to_html_report(&self, profile: &HealthProfile) -> Result<String> {
        let mut html = String::new();

        html.push_str(r#"<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>WIA Health Report</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; }
        h1 { color: #2c3e50; }
        h2 { color: #34495e; border-bottom: 2px solid #3498db; padding-bottom: 10px; }
        table { border-collapse: collapse; width: 100%; margin: 20px 0; }
        th, td { border: 1px solid #ddd; padding: 12px; text-align: left; }
        th { background-color: #3498db; color: white; }
        tr:nth-child(even) { background-color: #f2f2f2; }
        .header { text-align: center; margin-bottom: 40px; }
        .footer { margin-top: 40px; text-align: center; font-size: 12px; color: #7f8c8d; }
        .metric { font-size: 24px; font-weight: bold; color: #2c3e50; }
        .unit { font-size: 14px; color: #7f8c8d; }
    </style>
</head>
<body>
"#);

        // Header
        html.push_str(&format!(r#"
<div class="header">
    <h1>WIA Health Report</h1>
    <p>Subject ID: {}</p>
    <p>Generated: {}</p>
</div>
"#, profile.subject.id, Utc::now().format("%Y-%m-%d %H:%M UTC")));

        // Subject Information
        html.push_str("<h2>Subject Information</h2>\n<table>\n");
        html.push_str("<tr><th>Field</th><th>Value</th></tr>\n");
        html.push_str(&format!(
            "<tr><td>ID</td><td>{}</td></tr>\n",
            profile.subject.id
        ));
        if let Some(year) = profile.subject.birth_year {
            html.push_str(&format!(
                "<tr><td>Birth Year</td><td>{}</td></tr>\n",
                year
            ));
        }
        if let Some(ref sex) = profile.subject.biological_sex {
            html.push_str(&format!(
                "<tr><td>Biological Sex</td><td>{:?}</td></tr>\n",
                sex
            ));
        }
        html.push_str("</table>\n");

        // Biomarkers
        if let Some(ref biomarkers) = profile.biomarkers {
            html.push_str("<h2>Biomarkers</h2>\n");

            // Aging Clocks
            if let Some(ref clocks) = biomarkers.aging_clocks {
                html.push_str("<h3>Aging Clocks</h3>\n<table>\n");
                html.push_str("<tr><th>Marker</th><th>Value</th><th>Unit</th></tr>\n");
                if let Some(age) = clocks.chronological_age {
                    html.push_str(&format!(
                        "<tr><td>Chronological Age</td><td class='metric'>{:.1}</td><td class='unit'>years</td></tr>\n",
                        age
                    ));
                }
                if let Some(age) = clocks.biological_age {
                    html.push_str(&format!(
                        "<tr><td>Biological Age</td><td class='metric'>{:.1}</td><td class='unit'>years</td></tr>\n",
                        age
                    ));
                }
                if let Some(delta) = clocks.age_delta {
                    let color = if delta < 0.0 { "green" } else { "red" };
                    html.push_str(&format!(
                        "<tr><td>Age Delta</td><td class='metric' style='color:{}'>{:+.1}</td><td class='unit'>years</td></tr>\n",
                        color, delta
                    ));
                }
                html.push_str("</table>\n");
            }

            // Inflammatory Markers
            if let Some(ref inflammatory) = biomarkers.inflammatory_markers {
                html.push_str("<h3>Inflammatory Markers</h3>\n<table>\n");
                html.push_str("<tr><th>Marker</th><th>Value</th><th>Unit</th></tr>\n");
                if let Some(ref crp) = inflammatory.crp {
                    html.push_str(&format!(
                        "<tr><td>C-Reactive Protein (CRP)</td><td class='metric'>{:.2}</td><td class='unit'>{}</td></tr>\n",
                        crp.value, crp.unit
                    ));
                }
                html.push_str("</table>\n");
            }

            // Metabolic Markers
            if let Some(ref metabolic) = biomarkers.metabolic_markers {
                html.push_str("<h3>Metabolic Markers</h3>\n<table>\n");
                html.push_str("<tr><th>Marker</th><th>Value</th><th>Unit</th></tr>\n");
                if let Some(ref glucose) = metabolic.glucose {
                    html.push_str(&format!(
                        "<tr><td>Fasting Glucose</td><td class='metric'>{:.1}</td><td class='unit'>{}</td></tr>\n",
                        glucose.value, glucose.unit
                    ));
                }
                if let Some(ref hba1c) = metabolic.hba1c {
                    html.push_str(&format!(
                        "<tr><td>HbA1c</td><td class='metric'>{:.1}</td><td class='unit'>{}</td></tr>\n",
                        hba1c.value, hba1c.unit
                    ));
                }
                html.push_str("</table>\n");
            }
        }

        // Footer
        html.push_str(r#"
<div class="footer">
    <p>WIA Health Standard v1.0.0</p>
    <p>弘益人間 - Benefit All Humanity</p>
</div>
</body>
</html>
"#);

        Ok(html)
    }
}

impl Default for ExportFormatAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl IntegrationAdapter for ExportFormatAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::Export
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
        Ok(())
    }

    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities {
            can_import: false,
            can_export: true,
            can_stream: false,
            supports_auth: false,
            data_types: vec![
                "json".to_string(),
                "csv".to_string(),
                "html".to_string(),
                "pdf".to_string(),
            ],
        }
    }
}

#[async_trait]
impl ExportAdapter for ExportFormatAdapter {
    async fn export(&self, profile: &HealthProfile, options: ExportOptions) -> Result<ExportResult> {
        if !self.initialized {
            return Err(HealthError::adapter("Export adapter not initialized"));
        }

        let output = match options.format {
            ExportFormat::Json => self.to_json(profile, true)?,
            ExportFormat::Csv => self.to_csv(profile)?,
            ExportFormat::Pdf => self.to_html_report(profile)?,
            _ => self.to_json(profile, true)?,
        };

        // In a real implementation, write to file or return bytes
        let _ = output;

        Ok(ExportResult {
            success: true,
            records_exported: 1,
            destination: options.destination.unwrap_or_else(|| "memory".to_string()),
            resource_ids: vec![profile.id.to_string()],
            exported_at: Utc::now(),
        })
    }

    async fn export_data(&self, data: ExportData, _options: ExportOptions) -> Result<ExportResult> {
        if !self.initialized {
            return Err(HealthError::adapter("Export adapter not initialized"));
        }

        Ok(ExportResult {
            success: true,
            records_exported: 1,
            destination: "memory".to_string(),
            resource_ids: vec![data.data_type],
            exported_at: Utc::now(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::*;
    use uuid::Uuid;
    use chrono::NaiveDate;

    fn create_test_profile() -> HealthProfile {
        HealthProfile {
            id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            subject: Subject {
                id: Uuid::new_v4(),
                anonymized_id: Some("test-001".to_string()),
                birth_year: Some(1985),
                biological_sex: Some(BiologicalSex::Male),
                ethnicity: None,
                consent: None,
            },
            biomarkers: Some(BiomarkerProfile {
                aging_clocks: Some(AgingClocks {
                    chronological_age: Some(40.0),
                    biological_age: Some(37.5),
                    clock_type: None,
                    age_delta: Some(-2.5),
                    confidence: Some(0.92),
                    calculated_at: Some(Utc::now()),
                    algorithm: None,
                }),
                inflammatory_markers: Some(InflammatoryMarkers {
                    crp: Some(Measurement {
                        value: 1.5,
                        unit: "mg/L".to_string(),
                        reference_range: None,
                        timestamp: Some(Utc::now()),
                        method: None,
                        laboratory: None,
                        flags: None,
                        notes: None,
                    }),
                    ..Default::default()
                }),
                metabolic_markers: Some(MetabolicMarkers {
                    glucose: Some(Measurement {
                        value: 95.0,
                        unit: "mg/dL".to_string(),
                        reference_range: None,
                        timestamp: Some(Utc::now()),
                        method: None,
                        laboratory: None,
                        flags: None,
                        notes: None,
                    }),
                    ..Default::default()
                }),
                ..Default::default()
            }),
            genomics: None,
            epigenetics: None,
            telomeres: None,
            digital_twin: None,
            interventions: None,
            metadata: Metadata {
                created_at: Utc::now(),
                updated_at: Some(Utc::now()),
                source: None,
                version: "1.0.0".to_string(),
            },
        }
    }

    #[test]
    fn test_export_to_json() {
        let adapter = ExportFormatAdapter::new();
        let profile = create_test_profile();

        let json = adapter.to_json(&profile, true).unwrap();
        assert!(json.contains("biomarkers"));
        assert!(json.contains("subject"));
    }

    #[test]
    fn test_export_to_csv() {
        let adapter = ExportFormatAdapter::new();
        let profile = create_test_profile();

        let csv = adapter.to_csv(&profile).unwrap();
        assert!(csv.contains("category,field,value"));
        assert!(csv.contains("aging_clocks,biological_age"));
    }

    #[test]
    fn test_export_to_html() {
        let adapter = ExportFormatAdapter::new();
        let profile = create_test_profile();

        let html = adapter.to_html_report(&profile).unwrap();
        assert!(html.contains("<!DOCTYPE html>"));
        assert!(html.contains("WIA Health Report"));
        assert!(html.contains("Biomarkers"));
    }
}
