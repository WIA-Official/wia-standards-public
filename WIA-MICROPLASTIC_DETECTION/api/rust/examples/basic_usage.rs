//! Basic usage example for WIA Microplastic Detection SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_microplastic_detection::*;
use chrono::Utc;
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<()> {
    println!("WIA Microplastic Detection SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    // Create detection configuration
    let config = DetectionConfig {
        min_size_micrometers: 1.0,
        max_depth_meters: 100.0,
        detection_threshold: 0.8,
        sampling_rate_hz: 10.0,
    };

    // Validate configuration
    validators::validate_config(&config)?;
    println!("✓ Configuration validated");

    // Create sample particles
    let particles = vec![
        MicroplasticParticle {
            id: Uuid::new_v4(),
            detected_at: Utc::now(),
            location: GeoLocation {
                latitude: 37.7749,
                longitude: -122.4194,
                accuracy_meters: Some(10.0),
            },
            size_micrometers: 5.0,
            particle_type: PlasticType::Polyethylene,
            concentration: 0.05,
            depth_meters: 10.0,
            metadata: None,
        },
        MicroplasticParticle {
            id: Uuid::new_v4(),
            detected_at: Utc::now(),
            location: GeoLocation {
                latitude: 37.7750,
                longitude: -122.4195,
                accuracy_meters: Some(10.0),
            },
            size_micrometers: 3.0,
            particle_type: PlasticType::Polypropylene,
            concentration: 0.03,
            depth_meters: 15.0,
            metadata: None,
        },
    ];

    // Analyze particles
    let analysis = utils::create_analysis(particles);
    println!("✓ Analysis complete:");
    println!("  Total particles: {}", analysis.total_particles);
    println!("  Average concentration: {:.4}", analysis.average_concentration);
    println!("  Dominant type: {:?}", analysis.dominant_type);

    Ok(())
}
