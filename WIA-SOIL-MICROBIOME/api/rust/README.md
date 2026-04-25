# WIA-SOIL-MICROBIOME Rust SDK

[![Crates.io](https://img.shields.io/crates/v/wia-soil-microbiome-sdk.svg)](https://crates.io/crates/wia-soil-microbiome-sdk)
[![Documentation](https://docs.rs/wia-soil-microbiome-sdk/badge.svg)](https://docs.rs/wia-soil-microbiome-sdk)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Rust SDK for the WIA-SOIL-MICROBIOME Standard - Soil Health and Microbiome Analysis

> **弘益人間 (Benefit All Humanity)**

## Features

- **Complete Type Safety**: Fully typed Rust structs for all API endpoints
- **Async/Await**: Built on Tokio for high-performance async operations
- **WebSocket Streaming**: Real-time updates via WebSocket connections
- **Error Handling**: Comprehensive error types with detailed information
- **Diversity Calculations**: Built-in Shannon, Simpson, and richness calculations
- **Production Ready**: Includes retry logic, timeouts, and proper error handling

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
wia-soil-microbiome-sdk = "1.0.0"
tokio = { version = "1", features = ["full"] }
```

## Quick Start

### Basic Usage

```rust
use wia_soil_microbiome_sdk::{
    WiaSoilMicrobiomeClient, Config, Environment,
    SoilType, SamplingDepth, GeoLocation, SoilSample, SubmitSampleRequest,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create client
    let client = WiaSoilMicrobiomeClient::new(Config {
        api_key: "your-api-key".to_string(),
        environment: Environment::Production,
        ..Default::default()
    });

    // Submit a soil sample
    let sample = SoilSample {
        id: "sample-001".to_string(),
        collection_date: chrono::Utc::now(),
        location: GeoLocation {
            latitude: 37.7749,
            longitude: -122.4194,
            elevation: Some(50.0),
            description: Some("Test Farm".to_string()),
        },
        soil_type: SoilType::Agricultural,
        depth: SamplingDepth::Topsoil0To15Cm,
        properties: None,
        laboratory: None,
        metadata: None,
    };

    let request = SubmitSampleRequest {
        sample,
        analysis_methods: vec![
            AnalysisMethod::SixteenSRrnaSequencing,
        ],
    };

    let result = client.samples().submit(request).await?;
    println!("Sample submitted: {}", result.id);

    Ok(())
}
```

### Microbiome Analysis

```rust
use wia_soil_microbiome_sdk::AnalysisRequest;

// Get microbiome profile
let profile = client.microbiome()
    .get_profile("sample-001")
    .await?;

println!("Shannon diversity: {:?}", profile.diversity.shannon);
println!("Number of taxa: {}", profile.taxonomy.len());

// Get diversity metrics
let diversity = client.microbiome()
    .get_diversity("sample-001")
    .await?;

println!("Simpson index: {:?}", diversity.simpson);
```

### Soil Health Assessment

```rust
use wia_soil_microbiome_sdk::CalculateHealthIndexRequest;

// Calculate soil health index
let health_request = CalculateHealthIndexRequest {
    profile_id: "profile-001".to_string(),
    include_carbon: Some(true),
    include_nutrients: Some(true),
};

let health_index = client.health_index()
    .calculate(health_request)
    .await?;

println!("Soil health score: {}", health_index.score);
println!("Status: {:?}", health_index.status);
```

### Carbon Sequestration Tracking

```rust
use wia_soil_microbiome_sdk::CarbonSequestration;
use chrono::Utc;

// Submit carbon sequestration data
let carbon_data = CarbonSequestration {
    period_start: Utc::now() - chrono::Duration::days(365),
    period_end: Utc::now(),
    baseline: 50.0,
    current: 55.0,
    net_change: 5.0,
    annual_rate: 5.0,
    confidence: Some(0.85),
    method: Some("Direct measurement".to_string()),
};

let result = client.carbon()
    .submit_sequestration("sample-001", carbon_data)
    .await?;

println!("Carbon sequestration: {} tons/ha/year", result.annual_rate);

// Calculate carbon credits
let credits = client.carbon()
    .calculate_credits(
        "sample-001",
        "2023-01-01".to_string(),
        "2024-01-01".to_string(),
        100.0, // hectares
    )
    .await?;

println!("Carbon credits: {:?}", credits);
```

### WebSocket Streaming

```rust
use wia_soil_microbiome_sdk::{StreamingClient, Environment};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut streaming = StreamingClient::new("your-api-key", Environment::Production);

    // Connect to WebSocket
    streaming.connect().await?;

    // Subscribe to sample updates
    streaming.subscribe_sample("sample-001").await?;

    // Receive messages
    while let Some(message) = streaming.receive().await {
        println!("Received message: {:?}", message.header.message_type);

        // Handle different message types
        match message.header.message_type {
            MessageType::AnalysisResult => {
                println!("Analysis completed: {:?}", message.payload);
            }
            MessageType::AlertNotify => {
                println!("Alert received: {:?}", message.payload);
            }
            _ => {}
        }
    }

    Ok(())
}
```

### Local Diversity Calculations

```rust
use wia_soil_microbiome_sdk::{
    calculate_shannon_index,
    calculate_simpson_index,
    calculate_richness,
};

// Calculate diversity metrics locally
let abundances = vec![10.0, 20.0, 30.0, 40.0, 15.0];

let shannon = calculate_shannon_index(&abundances);
let simpson = calculate_simpson_index(&abundances);
let richness = calculate_richness(&abundances);

println!("Shannon diversity: {:.3}", shannon);
println!("Simpson diversity: {:.3}", simpson);
println!("Species richness: {}", richness);
```

## API Reference

### Client Methods

#### Samples API
- `samples().list(params)` - List all samples
- `samples().get(sample_id)` - Get a specific sample
- `samples().submit(request)` - Submit a new sample
- `samples().update(sample_id, sample)` - Update a sample
- `samples().delete(sample_id)` - Delete a sample
- `samples().get_status(sample_id)` - Get analysis status

#### Microbiome API
- `microbiome().get_profile(sample_id)` - Get microbiome profile
- `microbiome().request_analysis(request)` - Request new analysis
- `microbiome().get_diversity(sample_id)` - Get diversity metrics
- `microbiome().compare(sample_ids)` - Compare multiple samples
- `microbiome().get_functional_groups(sample_id)` - Get functional groups

#### Health Index API
- `health_index().calculate(request)` - Calculate soil health index
- `health_index().get(sample_id)` - Get health index
- `health_index().get_trends(sample_ids, start_date, end_date)` - Get health trends

#### Carbon API
- `carbon().submit_sequestration(sample_id, data)` - Submit carbon data
- `carbon().get_sequestration(sample_id)` - Get carbon data
- `carbon().calculate_credits(sample_id, start_date, end_date, area)` - Calculate carbon credits
- `carbon().get_trends(sample_ids, start_date, end_date)` - Get carbon trends

#### Interventions API
- `interventions().create(sample_id, intervention)` - Create intervention
- `interventions().list(sample_id)` - List interventions
- `interventions().update(sample_id, intervention_id, intervention)` - Update intervention
- `interventions().delete(sample_id, intervention_id)` - Delete intervention
- `interventions().get_effectiveness(sample_id, intervention_id)` - Get effectiveness

#### Reports API
- `reports().generate(sample_id)` - Generate complete report
- `reports().get(report_id)` - Get existing report
- `reports().list(params)` - List reports

## Error Handling

The SDK uses a comprehensive error type:

```rust
use wia_soil_microbiome_sdk::Error;

match client.samples().get("invalid-id").await {
    Ok(sample) => println!("Sample: {:?}", sample),
    Err(Error::NotFound(msg)) => println!("Sample not found: {}", msg),
    Err(Error::Authentication(msg)) => println!("Auth error: {}", msg),
    Err(Error::RateLimited { retry_after }) => {
        println!("Rate limited. Retry after: {:?}s", retry_after);
    }
    Err(e) => println!("Other error: {}", e),
}
```

## Configuration

```rust
use wia_soil_microbiome_sdk::{Config, Environment};

let config = Config {
    api_key: "your-api-key".to_string(),
    environment: Environment::Sandbox,
    timeout_secs: 60,
    max_retries: 5,
    base_url: None, // Optional custom URL
};

let client = WiaSoilMicrobiomeClient::new(config);
```

## Philosophy

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

This SDK is part of the WIA standards family, designed to promote soil health, biodiversity, and sustainable agriculture practices worldwide.

## License

MIT License - see LICENSE file for details

## Links

- [WIA Standards](https://wiastandards.com)
- [Documentation](https://docs.rs/wia-soil-microbiome-sdk)
- [GitHub](https://github.com/WIA-Official/wia-standards)
- [Crates.io](https://crates.io/crates/wia-soil-microbiome-sdk)

## Support

For issues and questions:
- GitHub Issues: https://github.com/WIA-Official/wia-standards/issues
- Email: support@wia.org

---

© 2025 WIA - World Certification Industry Association
