# WIA Health Rust SDK

**Health & Longevity Standards - Rust Implementation**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-1.70+-orange.svg)](https://www.rust-lang.org/)
[![Crates.io](https://img.shields.io/badge/crates.io-wia--health-blue.svg)](https://crates.io/crates/wia-health)

---

## Overview

High-performance Rust implementation of the WIA Health Standard for health and longevity data management.

### Features

- **Type-Safe**: Comprehensive types matching JSON Schema definitions
- **Async Support**: Full async/await support with Tokio
- **Validation**: Built-in data validation
- **Serialization**: JSON serialization with serde
- **Digital Twin**: Simulation engine for health predictions
- **Real-Time Protocol**: WebSocket streaming for live health data
- **Ecosystem Integration**: FHIR R4, HealthKit, Health Connect adapters
- **Multi-Format Export**: JSON, CSV, HTML/PDF export
- **Cross-Platform**: WebAssembly, Python, and Node.js bindings available

---

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-health = "1.0"
```

With optional features:

```toml
[dependencies]
wia-health = { version = "1.0", features = ["wasm"] }
```

---

## Quick Start

```rust
use wia_health::prelude::*;
use uuid::Uuid;
use chrono::NaiveDate;

fn main() -> Result<()> {
    // Create a subject
    let subject = Subject {
        id: Uuid::new_v4(),
        anonymized_id: Some("patient-001".to_string()),
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
    };

    // Build a health profile
    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .build()?;

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&profile)?;
    println!("{}", json);

    Ok(())
}
```

---

## Core Types

### Health Profile

The root type containing all health data:

```rust
pub struct HealthProfile {
    pub id: Uuid,
    pub version: String,
    pub subject: Subject,
    pub biomarkers: Option<BiomarkerProfile>,
    pub genomics: Option<GenomicProfile>,
    pub epigenetics: Option<EpigeneticProfile>,
    pub telomeres: Option<TelomereProfile>,
    pub digital_twin: Option<DigitalTwinProfile>,
    pub interventions: Option<InterventionHistory>,
    pub metadata: Metadata,
}
```

### Biomarkers

```rust
// Create aging clock data
let aging_clocks = AgingClocks {
    chronological_age: Some(40.0),
    biological_age: Some(36.5),
    clock_type: Some(AgingClockType::GrimAge),
    age_delta: Some(-3.5),
    confidence: Some(0.92),
    calculated_at: Some(Utc::now()),
    algorithm: None,
};

// Analyze age delta
let status = AgingClockCalculator::aging_status(-3.5);
// Returns: AgingStatus::Younger
```

### Telomeres

```rust
// Create telomere measurement
let measurement = TelomereMeasurement {
    id: Some(Uuid::new_v4()),
    average_length: TelomereLength {
        value: 7.5,
        unit: TelomereLengthUnit::Kilobases,
        percentile: None,
    },
    method: TelomereMeasurementMethod::QPCR,
    cell_type: Some(TelomereCellType::Leukocytes),
    timestamp: Utc::now(),
    ..Default::default()
};

// Estimate age from telomere length
let age = TelomereAnalyzer::estimate_age_from_length(7.5);
```

---

## Digital Twin Simulation

```rust
use wia_health::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    let profile = create_profile();
    let manager = DigitalTwinManager::new();

    // Create digital twin
    let twin = manager.create_twin(&profile)?;

    // Predict aging over 10 years
    let result = manager.predict_aging(&profile, 10.0).await?;

    println!("Predictions: {:?}", result.predictions);

    // Simulate intervention
    let intervention = Intervention {
        id: Uuid::new_v4(),
        category: InterventionCategory::Nutraceutical,
        name: "NMN Supplement".to_string(),
        status: InterventionStatus::Active,
        target_mechanism: Some(vec![
            TargetMechanism::Metabolic,
            TargetMechanism::Mitochondrial,
        ]),
        ..Default::default()
    };

    let result = manager
        .simulate_intervention(&profile, intervention, 5.0)
        .await?;

    Ok(())
}
```

---

## Analysis Functions

### Biomarker Analysis

```rust
// CRP inflammation check
let is_elevated = BiomarkerAnalyzer::is_elevated_crp(4.5); // true

// Glucose status
let status = BiomarkerAnalyzer::glucose_status(108.0);
// Returns: GlucoseStatus::Prediabetic

// HOMA-IR calculation
let homa_ir = BiomarkerAnalyzer::calculate_homa_ir(100.0, 10.0);
let ir_status = BiomarkerAnalyzer::interpret_homa_ir(homa_ir);
```

### Health Scores

```rust
// Calculate overall health score
let score = HealthScoreCalculator::calculate_overall_score(&profile);

// Calculate longevity index
let index = LongevityIndexCalculator::calculate(&profile)?;
println!("Longevity Index: {:.1}", index.overall);
```

---

## Communication Protocol

Real-time health data streaming with WebSocket:

```rust
use wia_health::protocol::*;
use wia_health::transport::*;
use chrono::Utc;

// Create protocol messages
let connect = MessageBuilder::connect(ConnectPayload {
    client_id: "health-app-001".to_string(),
    client_name: Some("WIA Health App".to_string()),
    client_version: Some("2.0.0".to_string()),
    capabilities: Some(vec!["biomarkers".to_string(), "alerts".to_string()]),
    subscriptions: None,
    options: Some(ConnectOptions::default()),
    auth: Some(AuthPayload {
        auth_type: "bearer".to_string(),
        token: "your-jwt-token".to_string(),
        refresh_token: None,
    }),
});

// Subscribe to biomarker streams
let subscribe = MessageBuilder::subscribe(vec![
    StreamSubscription {
        stream_type: "biomarkers".to_string(),
        filter: Some(StreamFilter {
            markers: Some(vec!["heart_rate".to_string(), "glucose".to_string()]),
            sources: None,
            min_quality: Some(0.8),
            clock_types: None,
        }),
        options: None,
    },
]);

// Send biomarker data
let biomarker = MessageBuilder::biomarker(BiomarkerPayload {
    stream_id: "stream-001".to_string(),
    subject_id: "patient-123".to_string(),
    sequence: 1,
    data: BiomarkerData {
        marker: "heart_rate".to_string(),
        value: 72.0,
        unit: "bpm".to_string(),
        timestamp: Utc::now().timestamp_millis(),
        source: None,
        quality: Some(0.95),
        metadata: None,
    },
});

// Configure WebSocket transport
let config = TransportConfig::with_url("wss://api.wia-health.org/ws")
    .timeout(5000)
    .reconnect_delay(1000);
```

### Protocol Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `connect` | C→S | Connection request |
| `connect_ack` | S→C | Connection acknowledgment |
| `subscribe` | C→S | Subscribe to streams |
| `biomarker` | S→C | Biomarker data point |
| `profile_update` | Both | Health profile changes |
| `simulation_result` | S→C | Digital twin results |
| `alert` | S→C | Health alerts |
| `ping/pong` | Both | Keep-alive |

---

## Error Handling

```rust
use wia_health::error::{HealthError, Result};

fn process_profile(data: &str) -> Result<HealthProfile> {
    let profile: HealthProfile = serde_json::from_str(data)?;

    // Validate age range
    if let Some(age) = profile.biomarkers
        .as_ref()
        .and_then(|b| b.aging_clocks.as_ref())
        .and_then(|c| c.biological_age)
    {
        if age < 0.0 || age > 150.0 {
            return Err(HealthError::out_of_range(
                "biological_age",
                age,
                0.0,
                150.0,
            ));
        }
    }

    Ok(profile)
}
```

---

## Ecosystem Integration

Phase 4 provides comprehensive integration with healthcare ecosystems:

### FHIR R4 Export

```rust
use wia_health::prelude::*;

// Convert profile to FHIR Bundle
let fhir_adapter = FhirAdapter::new();
let bundle = fhir_adapter.profile_to_bundle(&profile);
println!("Created {} FHIR resources", bundle.entry.len());

// Convert subject to FHIR Patient
let patient = fhir_adapter.subject_to_patient(&profile.subject);

// Convert measurement to FHIR Observation with LOINC codes
let observation = fhir_adapter.measurement_to_observation(
    &glucose_measurement,
    "Blood Glucose",
    LoincCodes::BLOOD_GLUCOSE,
    &subject_id,
);
```

### Wearable Device Integration

```rust
use wia_health::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize integration manager with wearable adapters
    let config = IntegrationConfig::default()
        .with_healthkit()
        .with_health_connect();

    let mut manager = IntegrationManager::new(config);
    manager.initialize().await?;

    // Import from HealthKit
    let result = manager
        .import_from_healthkit(ImportOptions::default())
        .await?;
    println!("Imported {} records", result.records_imported);

    Ok(())
}
```

### Multi-Format Export

```rust
use wia_health::prelude::*;

// Export to various formats
let export_adapter = ExportFormatAdapter::new();

// JSON
let json = export_adapter.to_json(&profile, true)?;

// CSV (flattened)
let csv = export_adapter.to_csv(&profile)?;

// HTML Report (for PDF conversion)
let html = export_adapter.to_html_report(&profile)?;
```

### Dashboard Integration

```rust
use wia_health::prelude::*;
use std::time::Duration;

let mut dashboard = DashboardAdapter::new();
dashboard.register_widget(Widget {
    id: "glucose-gauge".to_string(),
    widget_type: WidgetType::GaugeChart,
    data_source: "biomarkers.metabolic.glucose".to_string(),
    refresh_rate: Duration::from_secs(5),
    title: Some("Blood Glucose".to_string()),
    config: serde_json::json!({
        "min": 70,
        "max": 200,
        "thresholds": { "normal": 100, "warning": 140 }
    }),
});
```

### Integration Adapters

| Adapter | Purpose |
|---------|---------|
| `FhirAdapter` | FHIR R4 export with LOINC codes |
| `MockHealthKitAdapter` | Apple HealthKit import (mock for non-iOS) |
| `MockHealthConnectAdapter` | Google Health Connect import (mock for non-Android) |
| `DashboardAdapter` | Real-time dashboard widgets |
| `ExportFormatAdapter` | JSON, CSV, HTML/PDF export |
| `IntegrationManager` | Central adapter coordinator |

---

## Features

| Feature | Description |
|---------|-------------|
| `default` | Core functionality |
| `wasm` | WebAssembly support |
| `python` | Python bindings (PyO3) |
| `nodejs` | Node.js bindings (Neon) |
| `full` | All features |

---

## Examples

Run examples:

```bash
# Basic usage
cargo run --example basic_usage

# Digital twin simulation
cargo run --example digital_twin

# Communication protocol
cargo run --example protocol

# Ecosystem integration (FHIR, wearables, dashboard)
cargo run --example integration
```

---

## Testing

```bash
# Run all tests
cargo test

# Run with output
cargo test -- --nocapture

# Run specific test
cargo test test_aging_clock
```

---

## Project Structure

```
src/
├── lib.rs           # Library entry point
├── types.rs         # Type definitions
├── error.rs         # Error types
├── core/
│   ├── mod.rs
│   └── health.rs    # Core processing
├── adapters/
│   ├── mod.rs
│   └── simulator.rs # Digital twin simulator
├── integration/
│   ├── mod.rs       # Integration module
│   ├── adapter.rs   # Base adapter traits
│   ├── fhir.rs      # FHIR R4 adapter
│   ├── wearable.rs  # HealthKit/Health Connect
│   ├── dashboard.rs # Dashboard adapter
│   ├── export.rs    # Export format adapter
│   └── manager.rs   # Integration manager
├── protocol/
│   ├── mod.rs
│   ├── message.rs   # Protocol message types
│   └── handler.rs   # Connection handler
└── transport/
    ├── mod.rs
    └── websocket.rs # WebSocket transport

tests/
└── integration_test.rs

examples/
├── basic_usage.rs
├── digital_twin.rs
├── protocol.rs
└── integration.rs   # Ecosystem integration demo
```

---

## Performance

Benchmarks on Apple M2:

| Operation | Time |
|-----------|------|
| Profile serialization | ~50μs |
| Profile deserialization | ~80μs |
| Aging simulation (10yr) | ~1ms |
| Health score calculation | ~10μs |

---

## Related

- [Data Format Specification](../../spec/PHASE-1-DATA-FORMAT.md)
- [Communication Protocol](../../spec/PHASE-3-PROTOCOL.md)
- [Ecosystem Integration](../../spec/PHASE-4-INTEGRATION.md)
- [JSON Schemas](../../spec/schemas/)
- [WIA Standards Hub](https://wia.live/standards)

---

## License

MIT License - This standard belongs to humanity.

---

**弘益人間** - Benefit All Humanity 🤟🦀
