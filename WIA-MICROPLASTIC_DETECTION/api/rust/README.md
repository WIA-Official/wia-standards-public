# WIA Microplastic Detection Rust SDK

弘益人間 (Benefit All Humanity)

## Overview

Rust SDK for the WIA Microplastic Detection Standard. This SDK provides tools for detecting, analyzing, and tracking microplastics in marine environments.

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-microplastic-detection = "1.0.0"
```

## Quick Start

```rust
use wia_microplastic_detection::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create detection configuration
    let config = DetectionConfig {
        min_size_micrometers: 1.0,
        max_depth_meters: 100.0,
        detection_threshold: 0.8,
        sampling_rate_hz: 10.0,
    };

    // Create client
    let client = MicroplasticDetectionClient::new(
        "https://api.wia.org/microplastic".to_string()
    );

    // Detect microplastics
    let particles = client.detect(config).await?;

    // Analyze results
    let analysis = utils::create_analysis(particles);
    println!("Total particles: {}", analysis.total_particles);

    Ok(())
}
```

## Features

- **Detection**: Real-time microplastic particle detection
- **Analysis**: Comprehensive particle analysis and statistics
- **Validation**: Input validation for all data types
- **Async**: Full async/await support with Tokio
- **Type Safety**: Strong type system with Rust

## Plastic Types

- Polyethylene (PE)
- Polypropylene (PP)
- Polystyrene (PS)
- PVC
- PET
- Unknown

## API

### Client

- `MicroplasticDetectionClient::new(base_url)` - Create new client
- `client.detect(config)` - Detect microplastics
- `client.analyze(particles)` - Analyze particles

### Validators

- `validate_config(config)` - Validate detection configuration
- `validate_location(location)` - Validate geographic location
- `validate_particle(particle)` - Validate particle data

### Utils

- `calculate_average_concentration(particles)` - Calculate average
- `find_dominant_type(particles)` - Find most common type
- `create_analysis(particles)` - Create analysis result

## Testing

```bash
cargo test
```

## Examples

```bash
cargo run --example basic_usage
```

## License

MIT License - See LICENSE file for details

## Contact

- Website: https://wia.org
- Email: tech@wia.org
- GitHub: https://github.com/WIA-Official/wia-standards

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Benefit All Humanity)
