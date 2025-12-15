//! WIA Health Integration Example
//!
//! Demonstrates Phase 4 ecosystem integration features:
//! - FHIR R4 export
//! - Wearable device import (HealthKit, Health Connect)
//! - Dashboard visualization
//! - Multi-format export (JSON, CSV, HTML)

use chrono::{NaiveDate, Utc};
use uuid::Uuid;
use wia_health::prelude::*;
use wia_health::types::DataSource;

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Health Integration Demo ===\n");

    // Create a test health profile
    let profile = create_sample_profile();

    // 1. Initialize Integration Manager
    println!("1. Initializing Integration Manager...");
    let config = IntegrationConfig::default()
        .with_healthkit()
        .with_health_connect()
        .with_fhir("https://fhir.example.com/api", None);

    let mut manager = IntegrationManager::new(config);
    manager.initialize().await?;

    println!("   Available adapters: {:?}", manager.available_adapters());

    // 2. Import from wearable devices
    println!("\n2. Importing from wearable devices...");
    let import_result = manager
        .import_from_healthkit(ImportOptions::default())
        .await?;
    println!("   Imported {} records from HealthKit", import_result.records_imported);
    println!("   Data types: {:?}", import_result.data_types);

    // 3. Export to FHIR format
    println!("\n3. Exporting to FHIR R4 format...");
    let fhir_adapter = FhirAdapter::new();
    let bundle = fhir_adapter.profile_to_bundle(&profile);
    println!("   Created FHIR Bundle with {} entries", bundle.entry.len());

    // Show Patient resource
    if let Some(entry) = bundle.entry.first() {
        println!("   First entry: {:?}", entry.full_url);
    }

    // 4. Export to various formats
    println!("\n4. Exporting to various formats...");

    // JSON export
    let json_result = manager.export_to_json(&profile).await?;
    println!("   JSON export: {} records to {}",
        json_result.records_exported, json_result.destination);

    // CSV export
    let csv_result = manager.export_to_csv(&profile).await?;
    println!("   CSV export: {} records to {}",
        csv_result.records_exported, csv_result.destination);

    // 5. Direct format export examples
    println!("\n5. Direct format conversions...");
    let export_adapter = ExportFormatAdapter::new();

    // JSON
    let json = export_adapter.to_json(&profile, true)?;
    println!("   JSON length: {} bytes", json.len());

    // CSV
    let csv = export_adapter.to_csv(&profile)?;
    println!("   CSV preview:\n{}", csv.lines().take(5).collect::<Vec<_>>().join("\n"));

    // HTML Report
    let html = export_adapter.to_html_report(&profile)?;
    println!("   HTML report length: {} bytes", html.len());

    // 6. FHIR Conversion Examples
    println!("\n6. FHIR Conversion Examples...");

    // Subject to FHIR Patient
    let patient = fhir_adapter.subject_to_patient(&profile.subject);
    println!("   FHIR Patient ID: {}", patient.id);
    println!("   FHIR Patient Gender: {:?}", patient.gender);

    // Measurement to FHIR Observation
    if let Some(ref biomarkers) = profile.biomarkers {
        if let Some(ref metabolic) = biomarkers.metabolic_markers {
            if let Some(ref glucose) = metabolic.glucose {
                let observation = fhir_adapter.measurement_to_observation(
                    glucose,
                    "Blood Glucose",
                    LoincCodes::BLOOD_GLUCOSE,
                    &profile.subject.id.to_string(),
                );
                println!("   FHIR Observation LOINC: {:?}",
                    observation.code.coding.as_ref()
                        .and_then(|c| c.first())
                        .and_then(|c| c.code.as_ref()));
            }
        }
    }

    // 7. Dashboard Widget Configuration
    println!("\n7. Dashboard Widget Demo...");
    let mut dashboard = DashboardAdapter::new();
    dashboard.initialize(AdapterConfig::default()).await?;

    // Register widgets
    dashboard.register_widget(Widget {
        id: "glucose-gauge".to_string(),
        widget_type: WidgetType::GaugeChart,
        data_source: "biomarkers.metabolic.glucose".to_string(),
        refresh_rate: std::time::Duration::from_secs(5),
        title: Some("Blood Glucose".to_string()),
        config: serde_json::json!({
            "min": 70,
            "max": 200,
            "thresholds": {
                "normal": 100,
                "warning": 140,
                "critical": 180
            }
        }),
    });

    dashboard.register_widget(Widget {
        id: "age-chart".to_string(),
        widget_type: WidgetType::LineChart,
        data_source: "aging_clocks.biological_age".to_string(),
        refresh_rate: std::time::Duration::from_secs(60),
        title: Some("Biological Age Trend".to_string()),
        config: serde_json::json!({}),
    });

    println!("   Registered {} widgets", dashboard.widgets().len());

    // Create update for widget
    if let Some(widget) = dashboard.widgets().first() {
        if let Some(update) = dashboard.create_update(widget, &profile) {
            println!("   Dashboard update: {:?}", update.update_type);
        }
    }

    // 8. Wearable Sample Processing
    println!("\n8. Wearable Sample Processing...");
    let sample = WearableSample {
        data_type: WearableDataType::HeartRate,
        value: 72.0,
        unit: "bpm".to_string(),
        start_time: Utc::now(),
        end_time: None,
        source: Some("Apple Watch".to_string()),
        quality: Some(0.95),
        metadata: None,
    };

    let measurement = sample.to_measurement();
    println!("   Wearable: {} {} (quality: {:?})",
        sample.value, sample.unit, sample.quality);
    println!("   Converted to Measurement: {} {}",
        measurement.value, measurement.unit);

    // Cleanup
    manager.shutdown().await?;
    println!("\n=== Integration Demo Complete ===");
    println!("弘益人間 - Benefit All Humanity");

    Ok(())
}

fn create_sample_profile() -> HealthProfile {
    HealthProfile {
        id: Uuid::new_v4(),
        version: "1.0.0".to_string(),
        subject: Subject {
            id: Uuid::new_v4(),
            anonymized_id: Some("demo-patient-001".to_string()),
            birth_year: Some(1985),
            biological_sex: Some(BiologicalSex::Male),
            ethnicity: None,
            consent: Some(Consent {
                data_sharing: true,
                research: Some(true),
                consent_date: NaiveDate::from_ymd_opt(2025, 1, 1).unwrap(),
                version: Some("1.0".to_string()),
                expiration_date: None,
            }),
        },
        biomarkers: Some(BiomarkerProfile {
            aging_clocks: Some(AgingClocks {
                chronological_age: Some(40.0),
                biological_age: Some(37.5),
                clock_type: Some(AgingClockType::GrimAge),
                age_delta: Some(-2.5),
                confidence: Some(0.92),
                calculated_at: Some(Utc::now()),
                algorithm: None,
            }),
            inflammatory_markers: Some(InflammatoryMarkers {
                crp: Some(Measurement {
                    value: 1.2,
                    unit: "mg/L".to_string(),
                    reference_range: Some(ReferenceRange {
                        low: Some(0.0),
                        high: Some(3.0),
                        unit: Some("mg/L".to_string()),
                        population: None,
                    }),
                    timestamp: Some(Utc::now()),
                    method: Some("Immunoassay".to_string()),
                    laboratory: None,
                    flags: Some(vec![MeasurementFlag::Normal]),
                    notes: None,
                }),
                ..Default::default()
            }),
            metabolic_markers: Some(MetabolicMarkers {
                glucose: Some(Measurement {
                    value: 92.0,
                    unit: "mg/dL".to_string(),
                    reference_range: Some(ReferenceRange {
                        low: Some(70.0),
                        high: Some(100.0),
                        unit: Some("mg/dL".to_string()),
                        population: Some("Fasting glucose".to_string()),
                    }),
                    timestamp: Some(Utc::now()),
                    method: None,
                    laboratory: None,
                    flags: Some(vec![MeasurementFlag::Normal]),
                    notes: None,
                }),
                hba1c: Some(Measurement {
                    value: 5.4,
                    unit: "%".to_string(),
                    reference_range: Some(ReferenceRange {
                        low: Some(4.0),
                        high: Some(5.7),
                        unit: Some("%".to_string()),
                        population: None,
                    }),
                    timestamp: Some(Utc::now()),
                    method: None,
                    laboratory: None,
                    flags: Some(vec![MeasurementFlag::Normal]),
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
            source: Some(DataSource {
                system: "WIA Health Integration Demo".to_string(),
                version: Some("1.0.0".to_string()),
            }),
            version: "1.0.0".to_string(),
        },
    }
}
