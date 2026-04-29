//! Integration tests for WIA Microplastic Detection SDK

use wia_microplastic_detection::*;
use chrono::Utc;
use uuid::Uuid;

#[test]
fn test_config_validation() {
    let valid_config = DetectionConfig {
        min_size_micrometers: 1.0,
        max_depth_meters: 100.0,
        detection_threshold: 0.8,
        sampling_rate_hz: 10.0,
    };

    assert!(validators::validate_config(&valid_config).is_ok());

    let invalid_config = DetectionConfig {
        min_size_micrometers: -1.0,
        max_depth_meters: 100.0,
        detection_threshold: 0.8,
        sampling_rate_hz: 10.0,
    };

    assert!(validators::validate_config(&invalid_config).is_err());
}

#[test]
fn test_location_validation() {
    let valid_location = GeoLocation {
        latitude: 37.7749,
        longitude: -122.4194,
        accuracy_meters: Some(10.0),
    };

    assert!(validators::validate_location(&valid_location).is_ok());

    let invalid_location = GeoLocation {
        latitude: 100.0,
        longitude: -122.4194,
        accuracy_meters: Some(10.0),
    };

    assert!(validators::validate_location(&invalid_location).is_err());
}

#[test]
fn test_average_concentration() {
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

    let avg = utils::calculate_average_concentration(&particles);
    assert_eq!(avg, 0.04);
}

#[test]
fn test_analysis_creation() {
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
    ];

    let analysis = utils::create_analysis(particles);
    assert_eq!(analysis.total_particles, 1);
}
