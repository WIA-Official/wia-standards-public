# WIA-AGING SDK for Rust

🧓 **Rust SDK for the WIA-AGING Standard**

[![Crates.io](https://img.shields.io/crates/v/wia-aging-sdk)](https://crates.io/crates/wia-aging-sdk)
[![Documentation](https://docs.rs/wia-aging-sdk/badge.svg)](https://docs.rs/wia-aging-sdk)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

---

> **弘益人間 (Hongik Ingan)** - "Benefit All Humanity"

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-aging-sdk = "1.0"
tokio = { version = "1", features = ["full"] }
```

## Quick Start

```rust
use wia_aging_sdk::{
    WiaAgingClient, Config, Environment,
    CreateAssessmentRequest, BiologicalAgeMethod, Biomarker,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize client
    let client = WiaAgingClient::new(Config {
        api_key: "your-api-key".to_string(),
        environment: Environment::Production,
        ..Default::default()
    });

    // Create biomarkers
    let biomarkers = vec![
        Biomarker::new("WIA-AGE-001", 0.8, "mg/L")
            .with_name("C-Reactive Protein"),
        Biomarker::new("WIA-AGE-003", 4.5, "g/dL")
            .with_name("Albumin"),
        Biomarker::new("WIA-AGE-005", 92.0, "mg/dL")
            .with_name("Fasting Glucose"),
    ];

    // Create assessment
    let assessment = client.assessments()
        .create("profile_abc123", CreateAssessmentRequest {
            method: BiologicalAgeMethod::PhenotypicLevine,
            biomarkers,
        })
        .await?;

    println!("Biological Age: {:.1}", assessment.biological_age.value);
    println!("Age Difference: {:.1}", assessment.biological_age.age_difference.unwrap_or(0.0));
    println!("Health Score: {}/100", assessment.health_score);

    Ok(())
}
```

## Features

### Profile Management

```rust
// Create a profile
let profile = client.profiles()
    .create(CreateProfileRequest {
        chronological_age: 55.0,
        gender: Some(Gender::Female),
        date_of_birth: None,
    })
    .await?;

// List profiles
let profiles = client.profiles()
    .list(Some(PaginationParams {
        limit: Some(10),
        offset: Some(0),
    }))
    .await?;

// Get specific profile
let profile = client.profiles().get("profile_id").await?;
```

### Biomarker Submission

```rust
// Submit biomarkers
let result = client.biomarkers()
    .submit(
        "profile_id",
        vec![
            Biomarker::new("WIA-AGE-001", 0.8, "mg/L"),
            Biomarker::new("WIA-AGE-003", 4.5, "g/dL"),
        ],
        Some("laboratory"),
    )
    .await?;

// Get biomarker catalog
let catalog = client.biomarkers().catalog().await?;
```

### Intervention Tracking

```rust
use wia_aging_sdk::{CreateInterventionRequest, InterventionType};

// Create intervention
let intervention = client.interventions()
    .create("profile_id", CreateInterventionRequest {
        intervention_type: InterventionType::Supplement,
        name: "NMN".to_string(),
        dosage: Some("500mg".to_string()),
        frequency: Some("daily".to_string()),
        start_date: "2025-01-01".to_string(),
        notes: Some("Morning dose".to_string()),
    })
    .await?;
```

### Local Biological Age Calculation

```rust
use wia_aging_sdk::calculate_phenotypic_age;

let bio_age = calculate_phenotypic_age(
    55.0,               // chronological age
    Some(4.5),          // albumin (g/dL)
    Some(0.9),          // creatinine (mg/dL)
    Some(92.0),         // glucose (mg/dL)
    Some(1.0),          // CRP (mg/L)
    Some(28.0),         // lymphocyte (%)
);

println!("Biological Age: {:.1}", bio_age.value);
println!("Aging Rate: {:.2}x", bio_age.aging_rate.unwrap_or(1.0));
```

### Real-time Streaming

```rust
use wia_aging_sdk::streaming::{StreamingClient, StreamEvent};

let mut streaming = StreamingClient::new(config);
let mut events = streaming.connect().await?;

// Send biomarker updates
streaming.send_biomarkers("profile_id", biomarkers).await?;

// Handle events
while let Some(event) = events.recv().await {
    match event {
        StreamEvent::Connected => println!("Connected!"),
        StreamEvent::Authenticated => println!("Authenticated!"),
        StreamEvent::BiomarkerAck(ack) => {
            println!("Biomarkers accepted: {}", ack.accepted);
        }
        StreamEvent::Alert(alert) => {
            println!("Alert: {} - {}", alert.level, alert.message);
        }
        StreamEvent::Error(e) => eprintln!("Error: {}", e),
        StreamEvent::Disconnected => break,
    }
}
```

## Error Handling

```rust
use wia_aging_sdk::Error;

match client.profiles().get("invalid_id").await {
    Ok(profile) => println!("Found: {}", profile.id),
    Err(Error::NotFound(msg)) => println!("Profile not found: {}", msg),
    Err(Error::Authentication(msg)) => println!("Auth error: {}", msg),
    Err(Error::RateLimited { retry_after }) => {
        println!("Rate limited. Retry after {:?} seconds", retry_after);
    }
    Err(e) => println!("Other error: {}", e),
}
```

## Configuration

```rust
let config = Config {
    api_key: "your-api-key".to_string(),
    environment: Environment::Production,  // or Environment::Sandbox
    timeout_secs: 30,
    max_retries: 3,
    base_url: None,  // Optional override
};
```

## License

MIT License

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
